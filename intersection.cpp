#include "network.hpp"

static bool read_lane_ref(void *item, xmlTextReaderPtr reader)
{
    std::vector<lane_id> *lanev = reinterpret_cast<std::vector<lane_id>*>(item);

    char *ref = 0;
    int local_id;

    boost::fusion::vector<list_matcher<char*>,
        list_matcher<int> > vl(lm("ref", &ref),
                               lm("local_id", &local_id));

    if(!read_attributes(vl, reader))
       return false;

    if(local_id >= static_cast<int>(lanev->size()))
        lanev->resize(local_id+1);

    (*lanev)[local_id].sp = ref;

    return true;
}

static bool read_incoming(void *item, xmlTextReaderPtr reader)
{
    intersection *is = reinterpret_cast<intersection*>(item);
    xml_elt read[] =
        {{0,
          BAD_CAST "lane_ref",
          &(is->incoming),
          read_lane_ref}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "incoming"))
        return false;

    return true;;
}

static bool read_outgoing(void *item, xmlTextReaderPtr reader)
{
    intersection *is = reinterpret_cast<intersection*>(item);
    xml_elt read[] =
        {{0,
          BAD_CAST "lane_ref",
          &(is->outgoing),
          read_lane_ref}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "outgoing"))
        return false;

    return true;
}

static bool read_incident(void *item, xmlTextReaderPtr reader)
{
    intersection *is = reinterpret_cast<intersection*>(item);

    xml_elt read[] =
        {{0,
          BAD_CAST "incoming",
          item,
          read_incoming},
         {0,
          BAD_CAST "outgoing",
          item,
          read_outgoing}};

    bool status = read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "incident");
    if(!status)
        return false;

    return read[0].count == 1 && read[1].count == 1 && (is->incoming.size() + is->outgoing.size()) > 0 ;
}

static bool read_state(void *item, xmlTextReaderPtr reader)
{
    intersection *is = reinterpret_cast<intersection*>(item);

    is->states.push_back(intersection::state());

    intersection::state & new_state = is->states.back();

    new_state.in_states.resize(is->incoming.size());
    for(int i = 0; i < static_cast<int>(is->incoming.size()); ++i)
        new_state.in_states[i] = intersection::state::STOP;

    new_state.out_states.resize(is->outgoing.size());
    for(int i = 0; i < static_cast<int>(is->outgoing.size()); ++i)
        new_state.out_states[i] = intersection::state::STARVATION;

    return new_state.xml_read(reader);
}

static bool read_states(void *item, xmlTextReaderPtr reader)
{
    intersection *is = reinterpret_cast<intersection*>(item);

    if((is->incoming.size() + is->outgoing.size()) == 0)
        return false;

    xml_elt read[] =
        {{0,
          BAD_CAST "state",
          item,
          read_state}};

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "states"))
        return false;

    return read[0].count > 0;
}


static bool read_lane_pair(void *item, xmlTextReaderPtr reader)
{
    intersection::state *s = reinterpret_cast<intersection::state*>(item);

    int in, out;
    boost::fusion::vector<list_matcher<int>,
        list_matcher<int> > vl(lm("in_id",  &in),
                               lm("out_id", &out));

    if(!read_attributes(vl, reader))
       return false;

    s->in_states[in] = out;
    s->out_states[out] = in;

    return true;
}

bool intersection::state::xml_read(xmlTextReaderPtr reader)
{
    xml_elt read[] =
        {{0,
          BAD_CAST "lane_pair",
          this,
          read_lane_pair}};

    boost::fusion::vector<list_matcher<int>,
        list_matcher<float> > vl(lm("id",       &id),
                                 lm("duration", &duration));

    if(!read_attributes(vl, reader))
       return false;

    if(!read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "state"))
        return false;

    return true;
}

bool intersection::xml_read(xmlTextReaderPtr reader)
{
    xml_elt read[] =
        {{0,
          BAD_CAST "incident",
          this,
          read_incident},
         {0,
          BAD_CAST "states",
          this,
          read_states}};

    bool status = read_elements(reader, sizeof(read)/sizeof(read[0]), read, BAD_CAST "intersection");
    if(!status)
        return false;

    return read[0].count == 1 && read[1].count == 1;
}

int intersection::next_state()
{
    ++current_state;
    if(current_state >= static_cast<int>(states.size()))
        current_state = 0;

    return current_state;
}
