#include "network.hpp"
#include "arz.hpp"

bool network::load_from_xml(const char *filename)
{
    timer tim;
    tim.start();
    xmlTextReaderPtr reader = xmlReaderForFile(filename, NULL, XML_PARSE_XINCLUDE);
    if(!reader)
    {
        fprintf(stderr, "Couldn't open %s for reading\n", filename);
        return false;
    }

    int ret = xmlTextReaderRead(reader);
    while (ret == 1 && !is_opening_element(reader, "network"))
        ret = xmlTextReaderRead(reader);

    if(ret != 1)
    {
        if(ret == -1)
            fprintf(stderr, "Parsing error in %s\n", filename);
        else if(ret == 0)
            fprintf(stderr, "Couldn't find network element in %s\n", filename);

        xmlFreeTextReader(reader);
        return false;
    }

    xmlNodePtr np = xmlTextReaderCurrentNode(reader);
    if(!np)
    {
        fprintf(stderr, "Bad node pointer!\n");
        xmlFreeTextReader(reader);
        return false;
    }

    if(xmlXIncludeProcessTree(np) == -1)
    {
        fprintf(stderr, "Couldn't include!\n");
        xmlFreeTextReader(reader);
        return false;
    }

    bool status = xml_read(reader);

    xmlFreeTextReader(reader);
    xmlCleanupParser();

    tim.stop();
    printf("Loaded %s in %lf seconds\n", filename, tim.interval_S());

    return status;
}

bool network::xml_read(xmlTextReaderPtr reader)
{
    float version = 0.0f;

    boost::fusion::vector<list_matcher<float>,
        list_matcher<char*>,
        list_matcher<float> > vl(lm("version", &version),
                                 lm("name", &name),
                                 lm("gamma", &gamma_c));

    if(!read_attributes(vl, reader))
        return false;

    if(version != 1.0f)
    {
        fprintf(stderr, "Network version is %f, expected 1.0f!\n", version);
        return false;
    }

    inv_gamma = 1.0f/gamma_c;

    std::map<char*, int, ltstr> road_refs;
    std::map<char*, int, ltstr> lane_refs;
    std::map<char*, int, ltstr> intersection_refs;

    bool have_roads = false;
    bool have_lanes = false;
    bool have_intersections = false;

    int ret;
    do
    {
        ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return false;

        if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
        {
            const xmlChar *name = xmlTextReaderConstName(reader);
            if(!name)
                return false;

            if(xmlStrEqual(name, BAD_CAST "roads") && !have_roads)
                have_roads = read_sequence(roads, road_refs, reader, "road", "roads");
            else if(xmlStrEqual(name, BAD_CAST "lanes") && !have_lanes)
                have_lanes = read_sequence(lanes, lane_refs, reader, "lane", "lanes");
            else if(xmlStrEqual(name, BAD_CAST "intersections") && !have_intersections)
                have_intersections = read_sequence(intersections, intersection_refs, reader, "intersection", "intersections");
            else
                return false;
        }
    }
    while(ret == 1 && !is_closing_element(reader, "network"));

    printf("Read " SIZE_T_FMT " roads\n", roads.size());
    printf("Read " SIZE_T_FMT " lanes\n", lanes.size());
    printf("Read " SIZE_T_FMT " intersections\n", intersections.size());

    foreach(lane &la, lanes)
    {
        // convert end points
        if(la.start.end_type == lane_end::INTERSECTION)
            if(!la.start.inters.retrieve_ptr(intersections, intersection_refs))
                return false;

        if(la.end.end_type == lane_end::INTERSECTION)
            if(!la.end.inters.retrieve_ptr(intersections, intersection_refs))
                return false;

        // convert road_memberships
        road_intervals &ri = la.road_memberships;
        if(!ri.base_data.parent_road.retrieve_ptr(roads, road_refs))
            return false;

        foreach(road_intervals::entry &ent, ri.entries)
            if(!ent.data.parent_road.retrieve_ptr(roads, road_refs))
                return false;

        // convert left adjacencies
        {
            adjacency_intervals &ai = la.left;
            if(ai.base_data.neighbor.sp && !ai.base_data.neighbor.retrieve_ptr(lanes, lane_refs))
                return false;

            foreach(adjacency_intervals::entry &ent, ai.entries)
                if(ent.data.neighbor.sp && !ent.data.neighbor.retrieve_ptr(lanes, lane_refs))
                    return false;
        }

        // convert right adjacencies
        {
            adjacency_intervals &ai = la.right;
            if(ai.base_data.neighbor.sp && !ai.base_data.neighbor.retrieve_ptr(lanes, lane_refs))
                    return false;

            foreach(adjacency_intervals::entry &ent, ai.entries)
                if(ent.data.neighbor.sp && !ent.data.neighbor.retrieve_ptr(lanes, lane_refs))
                    return false;
        }
    }

    foreach(intersection &inter, intersections)
    {
        int i = 0;
        foreach(lane_id &lid, inter.incoming)
        {
            if(!lid.retrieve_ptr(lanes, lane_refs))
                return false;

            lid.dp->end.intersect_in_ref = i;
            ++i;
        }
        i = 0;
        foreach(lane_id &lid, inter.outgoing)
        {
            if(!lid.retrieve_ptr(lanes, lane_refs))
                return false;

            lid.dp->start.intersect_in_ref = i;
            ++i;
        }
    }

    typedef std::pair<char*const,int> reftype;
    foreach(reftype &pa, road_refs)
        free(pa.first);

    foreach(reftype &pa, lane_refs)
        free(pa.first);

    foreach(reftype &pa, intersection_refs)
        free(pa.first);

    foreach(lane &la, lanes)
        la.scale_offsets(LANE_WIDTH);

    foreach(intersection &inter, intersections)
        inter.build_shape(LANE_WIDTH);

    return ret == 1 && have_roads && have_lanes && have_intersections;
}

network::~network()
{
    free(name);
    free(lanes[0].data);
    free(lanes[0].rs);
    free(lanes[0].merge_states);
}

void network::prepare(float h)
{
    int total = 0;

    min_h = FLT_MAX;
    foreach(lane &la, lanes)
    {
        la.h = h;
        float len = la.calc_length();
        total += la.ncells = std::ceil(len/h);
        assert(la.ncells > 0);
        la.h = len/la.ncells;
        min_h = std::min(min_h, la.h);
    }

    printf("Allocating " SIZE_T_FMT " bytes for %d cells...", sizeof(q)*total, total);
    q *d = (q *) malloc(sizeof(q)*total);
    if(!d)
    {
        fprintf(stderr, "Failed!\n");
        exit(1);
    }
    else
        printf("Done\n");

    foreach(lane &la, lanes)
    {
        la.data = d;
        d += la.ncells;
    }

    printf("Allocating " SIZE_T_FMT " bytes for " SIZE_T_FMT " riemann solutions...", sizeof(riemann_solution)*(total + lanes.size()), total+lanes.size());
    riemann_solution *rs = (riemann_solution *) malloc(sizeof(riemann_solution)*(total + lanes.size()));
    if(!rs)
    {
        fprintf(stderr, "Failed!\n");
        exit(1);
    }
    else
        printf("Done\n");

    foreach(lane &la, lanes)
    {
        la.rs = rs;
        rs += la.ncells + 1;
    }

    printf("Allocating " SIZE_T_FMT " bytes for " SIZE_T_FMT " merge_states...", sizeof(merge_state)*(total + lanes.size()), total+lanes.size());
    merge_state *ms = (merge_state *) malloc(sizeof(merge_state)*(total + lanes.size()));
    if(!ms)
    {
        fprintf(stderr, "Failed!\n");
        exit(1);
    }
    else
        printf("Done\n");

    foreach(lane &la, lanes)
    {
        la.merge_states = ms;
        ms += la.ncells;
    }
}

void network::fill_from_carticles()
{
    foreach(lane &la, lanes)
        la.reset_data();

    foreach(lane &la, lanes)
        la.fill_from_carticles();

    foreach(lane &la, lanes)
        la.fill_y(gamma_c);
}

float network::sim_step()
{
    float maxspeed = 0.0f;
    foreach(lane &la, lanes)
    {
        float speed = la.collect_riemann(gamma_c, inv_gamma);
        maxspeed = std::max(maxspeed, speed);
    }

    foreach(intersection &is, intersections)
    {
        float speed = is.collect_riemann(gamma_c, inv_gamma);
        maxspeed = std::max(maxspeed, speed);
    }

    printf("maxspeed: %f\n", maxspeed);

    float dt = min_h/maxspeed;

    foreach(lane &la, lanes)
        la.update(dt);

    foreach(lane &la, lanes)
        la.advance_carticles(dt, gamma_c);

    foreach(lane &la, lanes)
        la.swap_carticles();

    return dt;
}

void network::calc_bounding_box()
{
    bb[0] = bb[2] = FLT_MAX;
    bb[1] = bb[3] = -FLT_MAX;

    foreach(const road &rd, roads)
    {
        foreach(const point &pt, rd.rep.points)
        {
            if(pt.x < bb[0])
                bb[0] = pt.x;
            else if(pt.x > bb[1])
                bb[1] = pt.x;
            if(pt.y < bb[2])
                bb[2] = pt.y;
            else if(pt.y > bb[3])
                bb[3] = pt.y;
        }
    }
    printf("bb[0] = %f bb[1] = %f\nbb[2] = %f bb[3] = %f\n", bb[0], bb[1], bb[2], bb[3]);
}
