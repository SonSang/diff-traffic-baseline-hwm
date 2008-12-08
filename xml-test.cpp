#include "xml-util.hpp"
#include <vector>

static void processNode(xmlTextReaderPtr reader)
{
    const xmlChar *name, *value;

    name = xmlTextReaderConstName(reader);
    if (name == NULL)
	name = BAD_CAST "--";

    value = xmlTextReaderConstValue(reader);

    printf("%d %d %s %d %d",
	    xmlTextReaderDepth(reader),
	    xmlTextReaderNodeType(reader),
	    name,
	    xmlTextReaderIsEmptyElement(reader),
	    xmlTextReaderHasValue(reader));
    if (value == NULL)
	printf("\n");
    else {
        if (xmlStrlen(value) > 40)
            printf(" %.40s...\n", value);
        else
	    printf(" %s\n", value);
    }
}

static int read_lane_pair(xmlTextReaderPtr reader)
{
    int in_id;
    int out_id;

    xmlChar *att = xmlTextReaderGetAttribute(reader, BAD_CAST "in_id");
    if(!att)
        return -1;
    sscanf((const char*) att, "%d", &in_id);
    free(att);

    att = xmlTextReaderGetAttribute(reader, BAD_CAST "out_id");
    if(!att)
        return -1;
    sscanf((const char*) att, "%d", &out_id);
    free(att);

    printf("Got lane_pair: %d %d\n", in_id, out_id);

    return 1;
}

static int read_state(xmlTextReaderPtr reader)
{
    int id;
    float duration;

    xmlChar *att = xmlTextReaderGetAttribute(reader, BAD_CAST "id");
    if(!att)
        return -1;
    sscanf((const char*) att, "%d", &id);
    free(att);

    att = xmlTextReaderGetAttribute(reader, BAD_CAST "duration");
    if(!att)
        return -1;
    sscanf((const char*) att, "%f", &duration);
    free(att);

    do
    {
        int ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return ret;
        if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
        {
            const xmlChar *name = xmlTextReaderConstName(reader);
            if(!name)
                return -1;
            printf("%s\n", (const char *) name);
            if(xmlStrEqual(name, BAD_CAST "lane_pair"))
                read_lane_pair(reader);
            else
                return -1;
        }
    }
    while(!is_closing_element(reader, "state"));

    return 1;
}

static int read_states(xmlTextReaderPtr reader)
{
    do
    {
        int ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return ret;

        if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
        {
            const xmlChar *name = xmlTextReaderConstName(reader);
            if(!name)
                return -1;
            if(xmlStrEqual(name, BAD_CAST "state"))
                read_state(reader);
            else
                return -1;
        }
    }
    while(!is_closing_element(reader, "states"));

    return 1;
}

static int read_lane(xmlTextReaderPtr reader)
{
    char * ref;
    int local_id;

    xmlChar *att = xmlTextReaderGetAttribute(reader, BAD_CAST "ref");
    if(!att)
        return -1;
    ref = strdupa((char *) att);
    free(att);

    att = xmlTextReaderGetAttribute(reader, BAD_CAST "local_id");
    if(!att)
        return -1;
    sscanf((const char*) att, "%d", &local_id);
    free(att);

    printf("Got lane: %s, %d\n", ref, local_id);

    return 1;
}

static int read_incoming(xmlTextReaderPtr reader)
{
    do
    {
        int ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return ret;
        if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
        {
            const xmlChar *name = xmlTextReaderConstName(reader);
            if(!name)
                return -1;
            if(xmlStrEqual(name, BAD_CAST "lane"))
                read_lane(reader);
            else
                return -1;
        }
    }
    while(!is_closing_element(reader, "incoming"));

    return 1;
}

static int read_outgoing(xmlTextReaderPtr reader)
{
    do
    {
        int ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return ret;
        if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
        {
            const xmlChar *name = xmlTextReaderConstName(reader);
            if(!name)
                return -1;
            if(xmlStrEqual(name, BAD_CAST "lane"))
                read_lane(reader);
            else
                return -1;
        }
    }
    while(!is_closing_element(reader, "outgoing"));

    return 1;
}

static int read_incident(xmlTextReaderPtr reader)
{
    bool have_incoming = false;
    bool have_outgoing = false;

    do
    {
        int ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return ret;
        if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
        {
            const xmlChar *name = xmlTextReaderConstName(reader);
            if(!name)
                return -1;
            if(xmlStrEqual(name, BAD_CAST "incoming") && !have_incoming)
                have_incoming = (read_incoming(reader) == 1);
            else if(xmlStrEqual(name, BAD_CAST "outgoing") && !have_outgoing)
                have_outgoing = (read_outgoing(reader) == 1);
            else
                return -1;
        }
    }
    while(!is_closing_element(reader, "outgoing"));

    return have_incoming && have_outgoing;
}

static int read_intersection(xmlTextReaderPtr reader)
{
    bool have_states   = false;
    bool have_incident = false;

    do
    {
        int ret = xmlTextReaderRead(reader);
        if(ret != 1)
            return ret;
        if(xmlTextReaderNodeType(reader) == XML_READER_TYPE_ELEMENT)
        {
            const xmlChar *name = xmlTextReaderConstName(reader);
            if(!name)
                return false;
            if(xmlStrEqual(name, BAD_CAST "states") && !have_states)
                have_states = (read_states(reader) == 1);
            else if(xmlStrEqual(name, BAD_CAST "incident") && !have_incident)
                have_incident = (read_incident(reader) == 1);
            else
                return false;
        }
    }
    while(!is_closing_element(reader, "intersection"));

    if(have_states)
        printf("I have states\n");
    else
        printf("I don't have states\n");

    if(have_incident)
        printf("I have incident\n");
    else
        printf("I don't have incident\n");

    return have_states && have_incident;
}

static void streamFile(const char *filename)
{
    xmlTextReaderPtr reader;
    int ret;

    reader = xmlReaderForFile(filename, NULL, 0);
    if (reader != NULL) {
        ret = xmlTextReaderRead(reader);
        while (ret == 1) {
            if(is_opening_element(reader, "intersection"))
                if(read_intersection(reader) == 0)
                {
                    fprintf(stderr, "failed to read intersection");
                    ret == -1;
                    break;
                }
            ret = xmlTextReaderRead(reader);
        }
        xmlFreeTextReader(reader);
        if (ret != 0) {
            fprintf(stderr, "%s : failed to parse\n", filename);
        }
    } else {
        fprintf(stderr, "Unable to open %s\n", filename);
    }
}

int main(int argc, char **argv)
{
    LIBXML_TEST_VERSION;

    streamFile(argv[1]);

    xmlCleanupParser();
    return(0);
}
