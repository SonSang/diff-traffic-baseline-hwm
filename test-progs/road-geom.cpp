#include <fstream>
#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <cfloat>
#include "network.hpp"

using namespace std;

#ifndef MSC_VER
#define _tmain main
#define _TCHAR char
#define strcpy_s(_dest, _num, _src) strncpy((_dest), (_src), (_num))
#define strncpy_s(_dest, _maxnum, _src, _num) strncpy((_dest), (_src), std::min(static_cast<size_t>(_maxnum),static_cast<size_t>(_num)))
#endif

// Vector of three dimensions

class point3{
public:
    point3(double,double,double);
    point3(double[3]);
    point3(point3*);
    point3(ifstream* infile);
    ~point3();
    void print();								//? Many of these function are no longer used
    void print(int);						// Print with given number of digits
    double pick(int);						// Find a coordinate's value.
    void set(int,double);				// Set a coordinate's value.
    void normalize();
    double length();
    void round();								// Round coordinates to nearest integer.
    point3* operator +  (point3*);
    point3* operator -  (point3*);
    point3* operator -  ();
    point3* operator *  (double);
    void operator += (point3*);
    void operator -= (point3*);
    int operator == (point3*);
    void printToFile(ofstream* outfile,double xoffset,double yoffset,double zoffset);	// Print vector like a vertex.
    void fprint(ofstream* outfile);
    void printToFile(ofstream* outfile);

    double x,y,z;};

void mem();
double dist(point3* a,point3* b);
int close(point3* a,point3* b);											// Determine if two vectors are close to each other.
int equal(point3* a,point3* b);
double orientation(point3* a,point3* b);
point3* multiply(double,point3*);										// Multiply a vector by a scalar.
double dot(point3* a,point3* b);										// Compute dot product
point3* cross(point3* a,point3* b);									// Compute cross product
point3* average(point3* a,point3* b);								// Compute an average
point3* copy(point3*);
point3* rotate(point3* p,double theta,point3* r);		// Rotate point p about axis r by the angle theta.
int large_dim(point3*);															// Return which coordinate has the greatest magnitude.
point3* interpolate(point3* start,point3* end,double t);

void print(point3* a);

point3* lose_dim(point3* loser,int coord);					// Remove one coordinate and return a 2D vector.

class paul_road;

class laneSet{
public:
    double start,end;
    int rightSide;
    vector<double> offsets;

    laneSet(double newStart,double newEnd,double newOffset,int newRightSide);
    laneSet(double newStart,double newEnd,vector<double> newOffset,int newRightSide);
    void draw(ofstream* outfile,paul_road*);
};

class paul_road{
public:
    vector<point3*> points;
    vector<laneSet*> left;
    vector<laneSet*> right;
    int sIntersect,eIntersect,over;
    paul_road* next;

    paul_road(vector<point3*> newPoints, int newOver, paul_road* next);
    void addLane(double start,double end,double offset);
    void draw(ofstream* outfile);
    void intersection(int sNotE);
    void overpass(double x);
    void addPoint(double x,double upOffset);
};

void addToSet(vector<laneSet*>& sets,double start,double end,double offset,int rightSide);
void drawSigns(ofstream* outfile);

int main(int argc, char* argv[])
{
    if(argc < 3)
    {
        fprintf(stderr, "Usage: %s <input network> <output file>\n", argv[0]);
        exit(1);
    }
    printf("%s version: %s\n", argv[0], hwm_version_string());

    network net;

    if(!net.load_from_xml(argv[1]))
    {
        fprintf(stderr, "Couldn't load %s\n", argv[1]);
        exit(1);
    }

    net.calc_bounding_box();
    point pt;
    pt.x = -(net.bb[0]+net.bb[1])*0.5f;
    pt.y = -(net.bb[2]+net.bb[3])*0.5f;
    pt.z = 0.0f;
    printf("Translating by %f %f %f\n", pt.x, pt.y, pt.z);
    net.translate(pt);

    std::map<const road*, paul_road*> road_map;

	paul_road* paul_roads = 0;

    foreach(const road &ro, net.roads)
    {
        vector<point3*> ppts;
        foreach(const point &pt, ro.rep.points)
        {
            ppts.push_back(new point3(pt.x, pt.y, pt.z));
        }
        int over = 0;
        paul_roads = new paul_road(ppts, over, paul_roads);

        assert(road_map.find(&ro) == road_map.end());

        road_map[&ro] = paul_roads;
    }

    foreach(const lane &la, net.lanes)
    {
        int sintersect = (la.start.end_type == lane_end::INTERSECTION);
        int eintersect = (la.end.end_type   == lane_end::INTERSECTION);

        const road_membership *rom = &(la.road_memberships.base_data);
        int p = -1;
        while(1)
        {
            std::map<const road*, paul_road*>::iterator parent_road = road_map.find(rom->parent_road.dp);

            assert(parent_road != road_map.end());

            float offset = rom->lane_position;
            int side = rom->interval[0] <  rom->interval[1];

            if(((sintersect) && (side)) || ((eintersect) && (!side)))
            {
                parent_road->second->intersection(1);
            }

            offset *= -1;
            if(offset < 0)
                offset += 0.5;
            else
                offset -= 0.5;

            parent_road->second->addLane(rom->interval[0],rom->interval[1],offset * 2.5);

            ++p;
            if(p >= static_cast<int>(la.road_memberships.entries.size()))
                break;
            rom = &(la.road_memberships.entries[p].data);
        }

    }

	//paul_roads->overpass("\"paul_road5\"",0.5);

    ofstream outfile(argv[2]);
    paul_roads->draw(&outfile);
    drawSigns(&outfile);

	return 0;
}

const int complexPart = 1;
const int simplePart = 1;

const double laneScale = 1.0;
const double interOffset = 0; //2.5 * laneScale;

paul_road::paul_road(vector<point3*> newPoints,int newOver,paul_road* newNext){
    points = newPoints;
	over = newOver;
	sIntersect = 0;
	eIntersect = 0;
	next = newNext;}

void paul_road::addLane(double start,double end,double offset){
    if(end > start)
        addToSet(right,start,end,offset,1);
    else
        addToSet(left ,end,start,offset,0);}

void insert(vector<laneSet*>& sets,int i,laneSet* insertion){
	vector<laneSet*>::iterator it = sets.begin();
	sets.insert(it+i,insertion);}

void addDividedLane(vector<laneSet*>& sets,double start,double end,double offset,int rightSide){
	int i = 0;
	while((i < (int)sets.size()) && (sets[i]->start < start))
		i++;
	if(i >= (int)sets.size())
		sets.push_back(new laneSet(start,end,offset,rightSide));
	else{
		if(sets[i]->start == start)
			sets[i]->offsets.push_back(offset);
		else
			insert(sets,i,new laneSet(start,end,offset,rightSide));}}

void divideLane(vector<laneSet*>& sets,double start,double end,double offset,int rightSide){
	for(int i = 0; i < (int)sets.size(); i++){
		if((start < sets[i]->start) && (sets[i]->start < end)){
			addDividedLane(sets,start,sets[i]->start,offset,rightSide);
            divideLane(sets,sets[i]->start,end  ,offset,rightSide);
			return;}
		if((start < sets[i]->end) && (sets[i]->end < end)){
			addDividedLane(sets,start,sets[i]->end,offset,rightSide);
            divideLane(sets,sets[i]->end,  end,offset,rightSide);
			return;}}

	addDividedLane(sets,start,end,offset,rightSide);
}

void addToSet(vector<laneSet*>& sets,double start,double end,double offset,int rightSide){
	for(int i = 0; i < (int)sets.size(); i++)
		if((sets[i]->start < start) && (start < sets[i]->end)){
			insert(sets,i+1,new laneSet(start,sets[i]->end,sets[i]->offsets,rightSide));
			sets[i]->end = start;}

	for(int i = 0; i < (int)sets.size(); i++)
		if((sets[i]->start < end) && (end < sets[i]->end)){
			insert(sets,i+1,new laneSet(end,sets[i]->end,sets[i]->offsets,rightSide));
			sets[i]->end = end;}

	divideLane(sets,start,end,offset,rightSide);}

laneSet::laneSet(double newStart, double newEnd, double newOffset,int newRightSide){
	start = newStart;
	end = newEnd;
	offsets.push_back(newOffset);
	rightSide = newRightSide;}

laneSet::laneSet(double newStart, double newEnd, vector<double> newOffsets,int newRightSide){
	start = newStart;
	end = newEnd;
	offsets = newOffsets;
	rightSide = newRightSide;}

int in(double start,double query,double end){
	return ((start <= query) && (query <= end));}

vector<point3*> leftSigns;
vector<point3*> rightSigns;

void addSign(point3* p,int sign){
	if(sign == 1)
		leftSigns.push_back(p);
	if(sign == 2)
		rightSigns.push_back(p);}

void clip(ofstream* outfile,double s1,double e1,double s2,double e2,double s3,double e3,point3* p3,point3* q3,int sign){
	if((e1 < s2) || (e2 < s1))
		return;

	double s,e;
	s = (in(s1,s2,e1)) ? s2 : s1;
	e = (in(s1,e2,e1)) ? e2 : e1;

	point3* result;
	result = interpolate(p3,q3,(s-s3)/(e3-s3));
	if(sign)
		addSign(result,sign);
	else{
		result->printToFile(outfile);
		delete result;}

	if(s2 == e2){
		if(sign)
			addSign(q3,sign);
		else
			q3->printToFile(outfile);}
	else{
		result = interpolate(p3,q3,(e-s3)/(e3-s3));
		result->printToFile(outfile);
		delete result;}}

void addPaul_RoadObject(ofstream* outfile,paul_road* r,double offset,const char* object,double left,double right,double startLane,double endLane,int dashed,double dashIn,double period,double cut){
	left  *= laneScale;
	right *= laneScale;
	point3* up = new point3(0,0,1);
	point3* start = 0;
	point3* end   = 0;
	point3* prevV =0;
	vector<point3*> starts;
	vector<point3*> ends;

	double prevAngle = -100;
	for(int i = 0; i < (int)r->points.size() - 1; i++){
		point3* v = *r->points[i+1] - r->points[i];
		v->normalize();

		double angle = atan(v->y / v->x);
		if(v->x < 0)
			angle += 3.141592654;
		else{
			if(angle < 0)
				angle += 2 * 3.141592654;}

		int sign = (abs(angle - prevAngle) > 3.141592654);

		if(prevAngle != -100){
			double offObj = sign ^ (angle - prevAngle < 0) ? right : left;
			*end += *prevV * ((offset - offObj) * tan((angle - prevAngle) / 2.0));
			ends.push_back(end);}

		point3* w = cross(v,up);
		w->normalize();
		w = *w * offset;
		start = *r->points[i  ] + w;
		double offObj = sign ^ (angle - prevAngle < 0) ? right : left;
		if(prevAngle != -100)
			*start += *v * (-(offset - offObj) * tan((angle - prevAngle) / 2.0));
		end   = *r->points[i+1] + w;

		starts.push_back(start);
		delete w;

		prevV = v;
		prevAngle = angle;}

	ends.push_back(end);

	double length = 0;
	for(int i = 0; i < (int)r->points.size() - 1; i++){
		point3* v = *r->points[i+1] - r->points[i];
		length += v->length();}

	double sL = startLane * length + cut;
	double eL =   endLane * length - cut;
	if(startLane == endLane)    // For single Objects
		eL = sL;
	else
		if(eL < sL)
			return;

	int sign = (strcmp(object,"leftSign") == 0) + 2 * (strcmp(object,"rightSign") == 0);
	if(!sign){
		*outfile<<object<<endl;
		if((dashed) && (dashIn == 0))
			*outfile<<"1"<<endl;
		else
			*outfile<<"2"<<endl;}

	if(!dashed){
		double pos = 0;
		for(int i = 0; i < (int)r->points.size() - 1; i++){
			point3* v = *r->points[i+1] - r->points[i];
			double vNorm = v->length();

			clip(outfile,sL,eL,pos,pos + vNorm,pos,pos + vNorm,starts[i],ends[i],sign);

			pos += vNorm;}}
	else{
		double pos  = 0;
		double base = 0;
		for(int i = 0; i < (int)r->points.size() - 1; i++){
			point3* v = *r->points[i+1] - r->points[i];
			double vNorm = v->length();

			if(sL == eL){
				if(in(base,sL,base+vNorm))
					clip(outfile,sL,eL,sL,sL,base,base + vNorm,starts[i],ends[i],sign);}
			else
				for(; pos - base < vNorm; pos += period)
					clip(outfile,sL,eL,pos,pos + dashIn,base,base + vNorm,starts[i],ends[i],sign);
			base += vNorm;}}

	if(!sign)
		*outfile<<"#"<<endl;}

void addPaul_RoadObject(ofstream* outfile,paul_road* r,double offset,const char* object,double left,double right,double startLane,double endLane,int dashed,double dashIn,double period){
	addPaul_RoadObject(outfile,r,offset,object,left,right,startLane,endLane,dashed,dashIn,period,0);}

void addPaul_RoadObject(ofstream* outfile,paul_road* paul_roads,double offset,const char* object,double left,double right,double startLane,double endLane){
	addPaul_RoadObject(outfile,paul_roads,offset,object,left,right,startLane,endLane,0,0,0,0);}

void paul_road::draw(ofstream* outfile){
	for(int i = 0; i < (int)left.size(); i++)
		left [i]->draw(outfile,this);
	for(int i = 0; i < (int)right.size(); i++)
		right[i]->draw(outfile,this);

	//if(complexPart)
	{
		if(sIntersect){
			if(left.size() > 0){
				for(int i = 0; i < (int)left[0]->offsets.size(); i++){
					if(complexPart)
						addPaul_RoadObject(outfile,this,left[0]->offsets[i],"$intersection",0,0,0,0,1,0,5,1.4 * laneScale + interOffset);
					if(simplePart)
						addPaul_RoadObject(outfile,this,left[0]->offsets[i],"$2crosswalk",0,0,0,0,1,0,5,0.4 * laneScale + interOffset);}}

			if(right.size() > 0){
				double max = right[0]->offsets[0];
				for(int i = 0; i < (int)right[0]->offsets.size(); i++){
					if(right[0]->offsets[i] > max)
						max = right[0]->offsets[i];
					if(complexPart)
						addPaul_RoadObject(outfile,this,right[0]->offsets[i],"$2light",0,0,0,0,1,0,5,1.4 * laneScale + interOffset);
					if(simplePart)
						addPaul_RoadObject(outfile,this,right[0]->offsets[i],"$2crosswalk",0,0,0,0,1,0,5,0.4 * laneScale + interOffset);}
				if(complexPart)
					addPaul_RoadObject(outfile,this,max+1.857 * laneScale,"$2lightSupport",0,0,0,0,1,0,5,1.4 * laneScale + interOffset);}}

		if(eIntersect){
			if(right.size() > 0){
				for(int i = 0; i < (int)right[right.size()-1]->offsets.size(); i++){
					if(complexPart)
						addPaul_RoadObject(outfile,this,right[right.size()-1]->offsets[i],"$intersection",0,0,1,1,1,0,5,-1.4 * laneScale  - interOffset);
					if(simplePart)
						addPaul_RoadObject(outfile,this,right[right.size()-1]->offsets[i],"$crosswalk",0,0,1,1,1,0,5,-0.4 * laneScale - interOffset);}}

			if(left.size() > 0){
				double min = left[left.size()-1]->offsets[0];
				for(int i = 0; i < (int)left[left.size()-1]->offsets.size(); i++){
					if(left[left.size()-1]->offsets[i] < min)
						min = left[left.size()-1]->offsets[i];
					if(complexPart)
						addPaul_RoadObject(outfile,this,left[left.size()-1]->offsets[i],"$light",0,0,1,1,1,0,5,-1.4  * laneScale - interOffset);
					if(simplePart)
						addPaul_RoadObject(outfile,this,left[left.size()-1]->offsets[i],"$crosswalk",0,0,1,1,1,0,5,-0.4  * laneScale - interOffset);}
				if(complexPart)
					addPaul_RoadObject(outfile,this,min-1.857 * laneScale,"$lightSupport",0,0,1,1,1,0,5,-1.4 * laneScale - interOffset);}}}

	if(next)
		next->draw(outfile);}

void laneSet::draw(std::ofstream *outfile, paul_road* r){
	int lowIndex  = 0;
	int highIndex = 0;
	double lowValue=FLT_MAX, highValue=-FLT_MAX;

	if((int)offsets.size() > 0){
		lowValue  = offsets[0];
		highValue = offsets[0];}

	for(int i = 1; i < (int)offsets.size(); i++){
		if(offsets[i] < lowValue){
			lowValue = offsets[i];
			lowIndex = i;}
		if(offsets[i] > highValue){
			highValue = offsets[i];
			highIndex = i;}}

	int endIndex = rightSide ? highIndex : lowIndex;
	int midIndex = rightSide ? lowIndex  : highIndex;

	if(!r->over){
		for(int i = 0; i < (int)offsets.size(); i++){
			if(simplePart)
				addPaul_RoadObject(outfile,r,offsets[i],"$road",-1.3,1.3,start,end);
			if(i == endIndex){
				if(simplePart){
					addPaul_RoadObject(outfile,r,offsets[i] - 2.5/2 * laneScale + 2.5 * rightSide * laneScale,"$white",-0.07,0.07,start,end);
					addPaul_RoadObject(outfile,r,offsets[i] - 5.0/2 * laneScale + 5.0 * rightSide * laneScale,"$road",-1.3,1.3,start,end);}
				if(complexPart){
					if(rightSide){
						addPaul_RoadObject(outfile,r,offsets[i] + 7.0/2 * laneScale ,"$leftBarrier" ,-0.056,0.05,start,end,0,0,0,10 * laneScale);
						//addPaul_RoadObject(outfile,r,offsets[i] + 7.4/2 * laneScale ,"$leftPost"    ,-0.04,0.056,start,end,1,0,5,3);
						addPaul_RoadObject(outfile,r,offsets[i] + 2.7 * laneScale   ,"leftSign"    ,0,0,start,end,1,0,75,40 * laneScale);}
					else{
						addPaul_RoadObject(outfile,r,offsets[i] - 7.0/2 * laneScale ,"$rightBarrier",-0.04,0.056,start,end,0,0,0,10 * laneScale);
						//addPaul_RoadObject(outfile,r,offsets[i] - 7.4/2 * laneScale ,"$rightPost"   ,-0.056,0.04,start,end,1,0,5,3);
						addPaul_RoadObject(outfile,r,offsets[i] - 2.7 * laneScale   ,"rightSign"   ,0,0,start,end,1,0,75,40 * laneScale);}}}
			else{
				if(simplePart)
					addPaul_RoadObject(outfile,r,offsets[i] - 2.5/2 * laneScale + 2.5 * rightSide * laneScale,"$dashed",-0.07,0.07,start,end);						// Using a texture
				//addPaul_RoadObject(outfile,r,offsets[i] - 2.5/2 * laneScale + 2.5 * rightSide * laneScale,"$white",-0.07,0.07,start,end,1,3,9);		// Geometrically
			}
			if(i == midIndex)
				if(simplePart)
					addPaul_RoadObject(outfile,r,offsets[i] + 2.25/2 * laneScale - 2.25 * rightSide * laneScale,"$orange",-0.07,0.07,start,end);}}

	// For the overpass

	else{
		for(int i = 0; i < (int)offsets.size(); i++){
			if(simplePart){
				addPaul_RoadObject(outfile,r,offsets[i],"$road",-1.3,1.3,start,end);
				addPaul_RoadObject(outfile,r,offsets[i],"$iBeams",-0.99,0.99,start,end);
				addPaul_RoadObject(outfile,r,offsets[i],"$underRoad",-1.55,1.55,start,end);
				addPaul_RoadObject(outfile,r,offsets[i],"$ramp",-1.5,1.5,start,end);}
			if(i == endIndex){
				if(simplePart)
					addPaul_RoadObject(outfile,r,offsets[i] - 2.5/2 * laneScale + 2.5 * rightSide * laneScale,"$white",-0.07,0.07,start,end);
				if(complexPart){
					if(rightSide){
						addPaul_RoadObject(outfile,r,offsets[i] + 1.25 * laneScale ,"$leftRailing"    ,-0.15,0.08,start,end,0,0,0,10 * laneScale);
						addPaul_RoadObject(outfile,r,offsets[i] + 1.25 * laneScale ,"$leftRailingPost",-0.15,0.08,start,end,1,0,5,10 * laneScale);}
					else{
						addPaul_RoadObject(outfile,r,offsets[i] - 1.25 * laneScale,"$rightRailing"   ,-0.08,0.15,start,end,0,0,0,10 * laneScale);
						addPaul_RoadObject(outfile,r,offsets[i] - 1.25 * laneScale,"$rightRailingPost",-0.08,0.15,start,end,1,0,5,10 * laneScale);}}}
			else{
				if(simplePart)
					addPaul_RoadObject(outfile,r,offsets[i] - 2.5/2 * laneScale + 2.5 * rightSide * laneScale,"$dashed",-0.07,0.07,start,end);					// Using a texture
				//addPaul_RoadObject(outfile,r,offsets[i] - 2.5/2 * laneScale + 2.5 * rightSide * laneScale,"$white",-0.07,0.07,start,end,1,3,9);				// Geometrically
			}
			if(i == midIndex)
				if(simplePart)
					addPaul_RoadObject(outfile,r,offsets[i] + 2.25/2 * laneScale - 2.25 * rightSide * laneScale,"$orange",-0.07,0.07,start,end);}}
}

void paul_road::intersection(int sNotE){
    if(sNotE)
        sIntersect = 1;
    else
        eIntersect = 1;}

void drawSigns(ofstream* outfile){
	vector<int> signChoice;
	int signNum = 9 + 2 + 4;

	for(int i = 0; i < (int)leftSigns.size() / 2; i++)
		signChoice.push_back(rand() % signNum + 1);

	*outfile<<"$1sign"<<endl<<"1"<<endl;	for(int i = 0; i < (int)leftSigns.size() / 2; i++)		if(signChoice[i] == 1){			leftSigns[2*i  ]->printToFile(outfile);			leftSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
	*outfile<<"$2sign"<<endl<<"1"<<endl;	for(int i = 0; i < (int)leftSigns.size() / 2; i++)		if(signChoice[i] == 2){			leftSigns[2*i  ]->printToFile(outfile);			leftSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
	*outfile<<"$3sign"<<endl<<"1"<<endl;	for(int i = 0; i < (int)leftSigns.size() / 2; i++)		if(signChoice[i] == 3){			leftSigns[2*i  ]->printToFile(outfile);			leftSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
	*outfile<<"$4sign"<<endl<<"1"<<endl;	for(int i = 0; i < (int)leftSigns.size() / 2; i++)		if(signChoice[i] == 4){			leftSigns[2*i  ]->printToFile(outfile);			leftSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
	*outfile<<"$5sign"<<endl<<"1"<<endl;	for(int i = 0; i < (int)leftSigns.size() / 2; i++)		if(signChoice[i] == 5){			leftSigns[2*i  ]->printToFile(outfile);			leftSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
	*outfile<<"$7sign"<<endl<<"1"<<endl;	for(int i = 0; i < (int)leftSigns.size() / 2; i++)		if(signChoice[i] == 6){			leftSigns[2*i  ]->printToFile(outfile);			leftSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
	*outfile<<"$8sign"<<endl<<"1"<<endl;	for(int i = 0; i < (int)leftSigns.size() / 2; i++)		if(signChoice[i] == 7){			leftSigns[2*i  ]->printToFile(outfile);			leftSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
	*outfile<<"$9sign"<<endl<<"1"<<endl;	for(int i = 0; i < (int)leftSigns.size() / 2; i++)		if(signChoice[i] == 8){			leftSigns[2*i  ]->printToFile(outfile);			leftSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
	*outfile<<"$6sign"<<endl<<"1"<<endl;	for(int i = 0; i < (int)leftSigns.size() / 2; i++)		if((signChoice[i] >= 9) && (signChoice[i] <= 11)) {			leftSigns[2*i  ]->printToFile(outfile);			leftSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;

	signChoice.clear();

	for(int i = 0; i < (int)rightSigns.size() / 2; i++)
		signChoice.push_back(rand() % signNum + 1);

	*outfile<<"$1sign01"<<endl<<"1"<<endl;	for(int i = 0; i < (int)rightSigns.size() / 2; i++)		if(signChoice[i] == 1){			rightSigns[2*i  ]->printToFile(outfile);			rightSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
	*outfile<<"$2sign01"<<endl<<"1"<<endl;	for(int i = 0; i < (int)rightSigns.size() / 2; i++)		if(signChoice[i] == 2){			rightSigns[2*i  ]->printToFile(outfile);			rightSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
	*outfile<<"$3sign01"<<endl<<"1"<<endl;	for(int i = 0; i < (int)rightSigns.size() / 2; i++)		if(signChoice[i] == 3){			rightSigns[2*i  ]->printToFile(outfile);			rightSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
	*outfile<<"$4sign01"<<endl<<"1"<<endl;	for(int i = 0; i < (int)rightSigns.size() / 2; i++)		if(signChoice[i] == 4){			rightSigns[2*i  ]->printToFile(outfile);			rightSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
	*outfile<<"$5sign01"<<endl<<"1"<<endl;	for(int i = 0; i < (int)rightSigns.size() / 2; i++)		if(signChoice[i] == 5){			rightSigns[2*i  ]->printToFile(outfile);			rightSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
	*outfile<<"$7sign01"<<endl<<"1"<<endl;	for(int i = 0; i < (int)rightSigns.size() / 2; i++)		if(signChoice[i] == 6){			rightSigns[2*i  ]->printToFile(outfile);			rightSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
	*outfile<<"$8sign01"<<endl<<"1"<<endl;	for(int i = 0; i < (int)rightSigns.size() / 2; i++)		if(signChoice[i] == 7){			rightSigns[2*i  ]->printToFile(outfile);			rightSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
	*outfile<<"$9sign01"<<endl<<"1"<<endl;	for(int i = 0; i < (int)rightSigns.size() / 2; i++)		if(signChoice[i] == 8){			rightSigns[2*i  ]->printToFile(outfile);			rightSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
	*outfile<<"$6sign01"<<endl<<"1"<<endl;	for(int i = 0; i < (int)rightSigns.size() / 2; i++)		if((signChoice[i] >= 9) && (signChoice[i] <= 11)){			rightSigns[2*i  ]->printToFile(outfile);			rightSigns[2*i+1]->printToFile(outfile);}	*outfile<<"#"<<endl;
}

void paul_road::addPoint(double x,double upOffset){
	double length = 0;
	for(int i = 0; i < (int)points.size() - 1; i++){
		point3* v = *points[i+1] - points[i];
		double vNorm = v->length();
		if(in(length,x,length+vNorm)){
			point3* p = interpolate(points[i],points[i+1],(x-length)/vNorm);
			*p += new point3(0,0,upOffset);
			vector<point3*>::iterator it = points.begin();
			points.insert(it+i+1,p);
			return;}
		length += vNorm;}}

void paul_road::overpass(double x)
{
    double length = 0;
    for(int i = 0; i < (int)points.size() - 1; i++){
        point3* v = *points[i+1] - points[i];
        length += v->length();}

    double addition[4];
    addition[0] = x * length - 15.0;
    addition[1] = x * length -  5.0;
    addition[2] = x * length +  5.0;
    addition[3] = x * length + 15.0;

    addPoint(addition[0],0);
    addPoint(addition[1],5);
    addPoint(addition[2],5);
    addPoint(addition[3],0);
}

point3::point3(double newx,double newy,double newz){
  x=newx;
  y=newy;
  z=newz;}

point3::point3(double newx[3]){
  x=newx[0];
  y=newx[1];
  z=newx[2];}

point3::point3(point3* a){
  x=a->x;
  y=a->y;
  z=a->z;}

point3::point3(ifstream* infile){
	*infile>>x;
	*infile>>y;
	*infile>>z;}


// Find a coordinate's value.

double point3::pick(int dim){
  if(dim==0) return x;
  if(dim==1) return y;
  if(dim==2) return z;
  return 0;}


// Set a coordinate's value.

void  point3::set(int dim,double value){
  if(dim==0) x=value;
  if(dim==1) y=value;
  if(dim==2) z=value;}

point3::~point3(){}

void point3::print(){
  cout<<"("<<x<<", "<<y<<", "<<z<<")"<<endl;}

void point3::fprint(ofstream* outfile){
	*outfile<<"["<<x<<", "<<y<<", "<<z<<"]"<<endl;}


// This function isn't being used, but should be.
// Round coordinates to nearest integer.

void point3::round(){
	int x0 = (int)((x+0.5-(x<0)));
	int y0 = (int)((y+0.5-(y<0)));
	int z0 = (int)((z+0.5-(z<0)));
	x = (double)x0;
	y = (double)y0;
	z = (double)z0;}

int equal(point3* a,point3* b){
  return ((a->x==b->x)&&(a->y==b->y)&&(a->z==b->z));}


// Determine if two vectors are close to each other.

int close(point3* a,point3* b){
	return (fabs(a->x-b->x)+fabs(a->y-b->y)+fabs(a->z-b->z) < 0.001);}


// Compute dot product

double dot(point3* a,point3* b){
  return a->x*b->x+a->y*b->y+a->z*b->z;}


// Compute cross product

point3* cross(point3* a,point3* b){
	return (new point3(a->y * b->z - a->z * b->y, a->z * b->x - a->x * b->z, a->x * b->y - a->y * b->x));}


// Multiply a vector by a scalar.

point3* multiply(double scalar,point3* a){
	return (new point3(scalar * a->x,scalar * a->y,scalar * a->z));}

point3* copy(point3* a){
	if(a==0)
		return a;
	else
		return new point3(a);}

void print(point3* a){
	if(a)
		a->print();}


// Return which coordinate has the greatest magnitude.

int large_dim(point3* n){
	if(fabs(n->x)>fabs(n->y)){
		if(fabs(n->x)>fabs(n->z))
			return 0;
		else
			return 2;}
	else{
		if(fabs(n->y)>fabs(n->z))
			return 1;
		else
			return 2;}}

double point3::length(){
	return sqrt(x*x+y*y+z*z);}

void point3::normalize(){
	double norm = length();
	x /= norm;
	y /= norm;
	z /= norm;}


point3* point3::operator +  (point3* a){ return (new point3(x + a->x,y + a->y,z + a->z));}
point3* point3::operator -  (point3* a){ return (new point3(x - a->x,y - a->y,z - a->z));}
void      point3::operator += (point3* a){ x += a->x; y += a->y; z += a->z;}
void      point3::operator -= (point3* a){ x -= a->x; y -= a->y; z += a->z;}

point3* point3::operator -  (){ return multiply(-1,this);}

point3* point3::operator *  (double a){ return multiply(a,this);}
int point3::operator ==  (point3* a){
	return ((x==a->x)&&(y==a->y)&&(z==a->z));}


// Remove one coordinate and return a 2D vector.

point3* lose_dim(point3* loser,int coord){
	switch(coord){
		case 0: return new point3(loser->y,loser->z,0); break;
		case 1: return new point3(loser->x,loser->z,0); break;
		case 2: return new point3(loser->x,loser->y,0); break;	}
	return 0;}


// Rotate point p about axis r by the angle theta.

point3* rotate(point3* p,double theta,point3* r)
{
   point3* q = new point3(0.0,0.0,0.0);
   double costheta,sintheta;

   //Normalise(&r);
   costheta = cos(theta);
   sintheta = sin(theta);

   q->x += (costheta + (1 - costheta) * r->x * r->x) * p->x;
   q->x += ((1 - costheta) * r->x * r->y - r->z * sintheta) * p->y;
   q->x += ((1 - costheta) * r->x * r->z + r->y * sintheta) * p->z;

   q->y += ((1 - costheta) * r->x * r->y + r->z * sintheta) * p->x;
   q->y += (costheta + (1 - costheta) * r->y * r->y) * p->y;
   q->y += ((1 - costheta) * r->y * r->z - r->x * sintheta) * p->z;

   q->z += ((1 - costheta) * r->x * r->z - r->y * sintheta) * p->x;
   q->z += ((1 - costheta) * r->y * r->z + r->x * sintheta) * p->y;
   q->z += (costheta + (1 - costheta) * r->z * r->z) * p->z;

   return(q);
}

// Print vector like a vertex.

void point3::printToFile(ofstream* outfile,double xoffset,double yoffset,double zoffset){
	*outfile<<1<<endl;
	*outfile<<"["<<x + xoffset<<","<<y + yoffset<<","<<z + zoffset<<"]"<<endl;
	*outfile<<"#"<<endl;}

void point3::printToFile(ofstream* outfile){
	*outfile<<"["<< x <<","<< y <<","<< z <<"]"<<endl;}


// Compute the distance between two points

double dist(point3* a,point3* b){
	double x = a->x - b->x;
	double y = a->y - b->y;
	double z = a->z - b->z;
	return sqrt(x * x + y * y + z * z);}

point3* average(point3* a,point3* b){
	return (new point3((a->x + b->x) / 2.0,(a->y + b->y) / 2.0,(a->z + b->z) / 2.0));}

point3* interpolate(point3* start,point3* end,double t){
	return (new point3(start->x * (1-t) + end->x * t, start->y * (1-t) + end->y * t, start->z * (1-t) + end->z * t));}
