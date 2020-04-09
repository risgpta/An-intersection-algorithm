#include<bits/stdc++.h>
using namespace std;

struct coord
{
    float x,y,z;
};

class triangleCollision
{
   public:

   coord _p1,_q1,_r1,_p2,_q2,_r2;

   inline void take_input();

   inline float calculateDet(float a,float b,float c,float d);

   inline coord obtainVector(coord p,float l,coord q);

   coord obtainIntersectionPoint(coord p,coord q,coord r,coord s);

   inline float dot(coord p,coord q);

   inline coord cross(coord v1,coord v2);

   inline coord getVector(coord q,coord p);

   float checkOrientation(coord p,coord q,coord r);

   inline float getDistance(coord p,coord q);

   inline float determinant3d(coord p,coord q, coord r,coord s);

   bool checkingPointinTriangle(coord _p2,coord _q2,coord _r2,coord _p1);

   coord barycentricCoord(coord v3,coord v4,float d00,float d01,float d11,float denom,coord _p1,coord _p2);

   inline void show(coord v);

   void anyIntersection(coord u1,coord u2,coord v1,coord v2,coord _p2,coord _q2,coord _r2);

   void findIntersection(coord _p1,coord _q1,coord _r1,coord _p2,coord _q2,coord _r2);// function containing anyIntersection function for a pair of edges

   void check_type_of_intersection(); // ie, No intersection or crossing intersection or coplanar intersection

   void checkCrossIntersection(coord _p1,coord _q1,coord _r1,coord _p2,coord _q2,coord _r2,float z1,float z2,float z3);

   void checkCoplanarIntersection(coord _p1,coord _q1,coord _r1,coord _p2,coord _q2,coord _r2);

   void settingPositionsaccordingly(coord &_p1,coord &_q1,coord &_r1,coord &_p2,coord &_q2,coord &_r2,float z1,float z2,float z3,float zz1,float zz2,float zz3);

   inline void swapPosition(coord *p,coord *q);

   void find3DTypesAndIntersections(coord _p1,coord _q1,coord _r1,coord _p2,coord _q2,coord _r2,coord v1,coord v2,coord v3,coord v4,coord n1,coord n2);

   void CoplanarDetectedRegion1(coord _p1,coord _q1,coord _r1,coord _p2,coord _q2,coord _r2,float d1,float d2,float d3);

   void CoplanarDetectedRegion2(coord _p1,coord _q1,coord _r1,coord _p2,coord _q2,coord _r2,float d1,float d2,float d3);

   coord baryIntersectionPoint(coord u1,coord u2,coord v1,coord v2);

   coord baryLine(coord a,coord b);

   inline void  ChangePermutationL(coord *p,coord *q,coord *r);

   inline void  ChangePermutationR(coord *p,coord *q,coord *r);
};

coord triangleCollision :: baryLine(coord a,coord b)
{
	coord s;
	s.x=(a.y*b.z-a.z*b.y);
	s.y=(a.z*b.x-a.x*b.z);
	s.z=(a.x*b.y-a.y*b.x);
	return s;
}

coord triangleCollision :: baryIntersectionPoint(coord u1,coord u2,coord v1,coord v2)
{
	coord e1=baryLine(u1,u2);
	coord e2=baryLine(v1,v2);
	coord s;
	s=baryLine(e1,e2);
	return s;
}

 inline void triangleCollision :: take_input()
 {
  cin>>_p1.x>>_p1.y>>_p1.z;
  cin>>_q1.x>>_q1.y>>_q1.z;
  cin>>_r1.x>>_r1.y>>_r1.z;
  cin>>_p2.x>>_p2.y>>_p2.z;
  cin>>_q2.x>>_q2.y>>_q2.z;
  cin>>_r2.x>>_r2.y>>_r2.z;
 }

inline float triangleCollision :: calculateDet(float a,float b,float c,float d)
{
    return a*d-c*b;
}

inline coord triangleCollision :: obtainVector(coord p,float l,coord q)
{
    coord s;
    s.x=p.x+l*q.x;
    s.y=p.y+l*q.y;
    s.z=p.z+l*q.z;
    return s;
}

coord triangleCollision :: obtainIntersectionPoint(coord p,coord q,coord r,coord s)
{
    float l =calculateDet(q.x,-s.x,q.y,-s.y)/calculateDet(r.x-p.x,-s.x,r.y-p.y,-s.y);
    coord i;
    i=obtainVector(p,l,q);
    return i;
}

inline float triangleCollision :: dot(coord p,coord q)
{
    return p.x*q.x+p.y*q.y+p.z*q.z;
}

inline coord triangleCollision :: getVector(coord q,coord p)
{
    coord s;
    s.x=q.x-p.x;
    s.y=q.y-p.y;
    s.z=q.z-p.z;
    return s;
}

inline coord triangleCollision :: cross(coord v1,coord v2)
{
    coord s;
    s.x=(v1.y*v2.z)-(v1.z*v2.y);
    s.y=-1*((v1.x*v2.z)-(v1.z*v2.x));
    s.z=(v1.x*v2.y-v1.y*v2.x);
    return s;
}

inline float triangleCollision :: checkOrientation(coord p,coord q,coord r)
{
    float f=(p.x-r.x)*(q.y-r.y)-(q.x-r.x)*(p.y-r.y);
    return f;
}

inline float triangleCollision :: getDistance(coord p,coord q)
{
  float z=(q.x-p.x)*(q.x-p.x)+(q.y-p.y)*(q.y-p.y);
  return z;
}

inline float triangleCollision :: determinant3d(coord p,coord q, coord r,coord s)
{
  p.x-=s.x;
  p.y-=s.y;
  p.z-=s.z;
  q.x-=s.x;
  q.y-=s.y;
  q.z-=s.z;
  r.x-=s.x;
  r.y-=s.y;
  r.z-=s.z;
  float z;
  z=p.x*(q.y*r.z-q.z*r.y)-p.y*(q.x*r.z-q.z*r.x)+p.z*(q.x*r.y-q.y*r.x);
  return z;
}

bool triangleCollision :: checkingPointinTriangle(coord _p2,coord _q2,coord _r2,coord _p1)
{
    float a1,a2,a3;
    // checking if _p1 lies inside, on edge or on vertex:
    a1=checkOrientation(_p1,_p2,_q2);
    a2=checkOrientation(_p1,_q2,_r2);
    a3=checkOrientation(_p1,_r2,_p2);
    if(a1>0 && a2>0 && a3>0)
      return true;
  return false;
}

coord triangleCollision :: barycentricCoord(coord v3,coord v4,float d00,float d01,float d11,float denom,coord _p1,coord _p2)
{
   coord v5=getVector(_p1,_p2);
   float d20=dot(v5,v3);
   float d21=dot(v5,v4);
   coord s;
 //  cout<<"bary:     ";
  // cout<<s.y<<" "<<s.z<<" \n";
   s.y=(d11*d20-d01*d21)*denom;
   s.z=(d00*d21-d01*d20)*denom;
   s.x=1.0f-s.y-s.z;
   return s;
}

inline void triangleCollision :: show(coord v)
{
  cout<<v.x<<" "<<v.y<<" "<<v.z<<"\n";
}

void triangleCollision :: anyIntersection(coord u1,coord u2,coord v1,coord v2,coord _p2,coord _q2,coord _r2)
{
//	show(v1);
//	show(v2);
 //   cout<<"\n\n";
	bool w1,w2,w3;
    coord s=baryIntersectionPoint(u1,u2,v1,v2);
    float zz=s.x+s.y+s.z;
    zz=1/zz;
    s.x*=zz;
    s.y*=zz;
    s.z*=zz;
    float w11=u1.x+u1.y+u1.z;
    float w22=u2.x+u2.y+u2.z;
    float t=(s.x-u2.x/w22)/(u1.x/w11-u2.x/w22);
  //  cout<<"s:     ";
   // 	show(s);
   // cout<<"t: =  "<<t<<"\n";
      if(s.x>=0.0f && s.y>=0.0f && s.z>=0.0f && t>=0.0f && t<=1.0f)
      {
        float xx,yy,zz;
        coord u1=s;
        xx=u1.x*_p2.x+u1.y*_q2.x+u1.z*_r2.x;
        yy=u1.x*_p2.y+u1.y*_q2.y+u1.z*_r2.y;
        zz=u1.x*_p2.z+u1.y*_q2.z+u1.z*_r2.z;
        cout<<"intersection point:        ";
        cout<<xx<<"  "<<yy<<" "<<zz<<"\n";
        return;
      }
}

void triangleCollision :: findIntersection(coord _p1,coord _q1,coord _r1,coord _p2,coord _q2,coord _r2)
{
   coord v3=getVector(_q2,_p2);
   coord v4=getVector(_r2,_p2);
   float d00=dot(v3,v3);
   float d01=dot(v3,v4);
   float d11=dot(v4,v4);
   float denom=d00*d11-d01*d01;
   denom=1.0f/denom;
   coord b1=barycentricCoord(v3,v4,d00,d01,d11,denom,_p1,_p2);
   coord b2=barycentricCoord(v3,v4,d00,d01,d11,denom,_q1,_p2);
   coord b3=barycentricCoord(v3,v4,d00,d01,d11,denom,_r1,_p2);
   coord p22,q22,r22;
   p22.x=1;
   p22.y=0.0f;
   p22.z=0.0f;
   q22.x=0.0f;
   q22.y=1;
   q22.z=0.0f;
   r22.x=0.0f;
   r22.y=0.0f;
   r22.z=1;
    if(b1.x*b2.x>0.0f && b1.y*b2.y>0.0f && b1.z*b2.z>0.0f)
    cout<<"no intersection for _p1 and _q1\n";
    else
    {
    //	show(_p1);show(_q1);
    //	cout<<":------\n";
	//	show(_p2);show(_q2);
        anyIntersection(b1,b2,p22,q22,_p2,_q2,_r2);
    //    show(_q2);show(_r2);
        anyIntersection(b1,b2,q22,r22,_p2,_q2,_r2);
    //    show(_r2);show(_p2);
		anyIntersection(b1,b2,r22,p22,_p2,_q2,_r2);
    }
    if(b3.x*b2.x>0.0f && b3.y*b2.y>0.0f && b3.z*b2.z>0.0f)
    cout<<"no intersection for _r1 and _q1\n";
    else
    {
    //	show(_q1);show(_r1);
    //	cout<<":------\n";
    //		show(_p2);show(_q2);
        anyIntersection(b2,b3,p22,q22,_p2,_q2,_r2);
    //     show(_q2);show(_r2);
        anyIntersection(b2,b3,q22,r22,_p2,_q2,_r2);
    //       show(_r2);show(_p2);
        anyIntersection(b2,b3,r22,p22,_p2,_q2,_r2);

    }
    if(b1.x*b3.x>0.0f && b1.y*b3.y>0.0f && b1.z*b3.z>0.0f)
    cout<<"no intersection for _p1 and _r1\n";
    else
    {
    //		show(_p1);show(_r1);
    //		cout<<":------\n";
    //			show(_p2);show(_q2);
        anyIntersection(b1,b3,p22,q22,_p2,_q2,_r2);
    //     show(_q2);show(_r2);
        anyIntersection(b1,b3,q22,r22,_p2,_q2,_r2);
    //       show(_r2);show(_p2);
        anyIntersection(b1,b3,r22,p22,_p2,_q2,_r2);
    }
}

void triangleCollision :: check_type_of_intersection()
{
  float z1,z2,z3;
  z1=determinant3d(_p2,_q2,_r2,_p1);
  z2=determinant3d(_p2,_q2,_r2,_q1);
  z3=determinant3d(_p2,_q2,_r2,_r1);
  int f=2;
 // cout<<z1<<" "<<z2<<" "<<z3<<"..deter...\n";
  if((z1>0.0f && z2>0.0f && z3>0.0f) || (z1<0.0f && z2<0.0f && z3<0.0f))
  f=0;
  if(z1<0.1f && z1>-0.1f  && z2<0.1f && z2>-0.1f && z3<0.1f && z3>-0.1f)
  f=1;
  if(f==0)
  {
    cout<<"No Intersection!";
    return;
  }
  // two vectors
  coord v3=getVector(_q2,_p2);
  coord v4=getVector(_r2,_p2);
  if(f==2)
  checkCrossIntersection(_p1,_q1,_r1,_p2,_q2,_r2,z1,z2,z3);
  else
  checkCoplanarIntersection(_p1,_q1,_r1,_p2,_q2,_r2);
}

inline void triangleCollision :: swapPosition(coord *p,coord *q)
{
	coord s;
	s=*p;
	*p=*q;
	*q=s;
	return;
}
inline void triangleCollision :: ChangePermutationL(coord *p,coord *q,coord *r)
{
  coord s;
  s=*p;
  *p=*q;
  *q=*r;
  *r=s;
  return;
}
inline void triangleCollision :: ChangePermutationR(coord *p,coord *q,coord *r)
{
  coord s;
  s=*r;
  *r=*q;
  *q=*p;
  *p=s;
  return;
}
void triangleCollision :: settingPositionsaccordingly(coord &_p1,coord &_q1,coord &_r1,coord &_p2,coord &_q2,coord &_r2,float z1,float z2,float z3,float zz1,float zz2,float zz3)
{
	 // one on plane T2, T1:
    if(z1==0.0f || z2==0.0f || z3==0.0f)
    {
      if(z2==0.0f)
      ChangePermutationL(&_p1,&_q1,&_r1);
      else if(z3==0.0f)
      ChangePermutationR(&_p1,&_q1,&_r1);
    }
    // one on plane T1, T2:
    if(zz1==0.0f || zz2==0.0f || zz3==0.0f)
    {
      if(zz2==0.0f)
    ChangePermutationL(&_p2,&_q2,&_r2);
      else if(zz3==0.0f)
	  ChangePermutationR(&_p2,&_q2,&_r2);
    }
    // to get _cout<<y1<<" "<<y2<<"\n";p1 on one side , two cases: z1>0.0f and z1<0.0f
    if(z1>0.0f)
    {
      if(z2>0.0f && z3<0.0f)
          ChangePermutationR(&_p1,&_q1,&_r1);
      else if(z2<0.0f && z3>0.0f)
            ChangePermutationL(&_p1,&_q1,&_r1);
    }
    else
    {
      if(z2>0.0f && z3<0.0f)
          ChangePermutationL(&_p1,&_q1,&_r1);
      else if(z2<0.0f && z3>0.0f)
            ChangePermutationR(&_p1,&_q1,&_r1);
    }
    // to get _p2 on one side , two cases: zz1>0.0f and zz1<0.0f
    if(zz1>0.0f)
    {
      if(zz2>0.0f && zz3<0.0f)
      ChangePermutationR(&_p2,&_q2,&_r2);
      else if(zz2<0.0f && zz3>0.0f)
      ChangePermutationL(&_p2,&_q2,&_r2);
    }
    else
    {
      if(zz2>0.0f && zz3<0.0f)
      ChangePermutationL(&_p2,&_q2,&_r2);
      else if(zz2<0.0f && zz3>0.0f)
      ChangePermutationR(&_p2,&_q2,&_r2);
    }

    // to get _p1 on positive side:
    float y1=determinant3d(_p2,_q2,_r2,_p1);
    if(y1<0.0f)
    swapPosition(&_q2,&_r2);
    // to get _p2 on positive side:
    float y2=determinant3d(_p1,_q1,_r1,_p2);
    if(y2<0.0f)
    swapPosition(&_q1,&_r1);
    return ;
}
void triangleCollision :: find3DTypesAndIntersections(coord _p1,coord _q1,coord _r1,coord _p2,coord _q2,coord _r2,coord v1,coord v2,coord v3,coord v4,coord n1,coord n2)
{

	 if(determinant3d(_p1,_r1,_q2,_p2)>0.0f)
      {
        if(determinant3d(_p1,_q1,_r2,_p2)>0.0f)
        {
            coord pp=getVector(_p2,_p1);
          float z1=dot(pp,n1);
          float z3=dot(v4,n1);
          float lam2=-z1/z3;
          coord s2=obtainVector(_p2,lam2,v4);
          cout<<s2.x<<" "<<s2.y<<" "<<s2.z<<"\n";
          coord ss=getVector(_p1,_p2);
          z1=dot(ss,n2);
          float z2=dot(v2,n2);
          float lam3=-z1/z2;
          coord s3=obtainVector(_p1,lam3,v2);
          cout<<s3.x<<" "<<s3.y<<" "<<s3.z<<"\n";;
          cout<<"k i l j\n";
        }
        else
        {
            coord pp=getVector(_p1,_p2);
          float z1=dot(pp,n2);
          float z2=dot(v1,n2);
          float z3=dot(v2,n2);
          float lam1=-z1/z2;
          float lam2=-z1/z3;
          coord s1=obtainVector(_p1,lam1,v1);
          coord s2=obtainVector(_p1,lam2,v2);
          cout<<s1.x<<" "<<s1.y<<" "<<s1.z<<"\n";
          cout<<s2.x<<" "<<s2.y<<" "<<s2.z<<"\n";
          cout<<"k i j l\n";
        }
      }
      else
      {
        if(determinant3d(_p1,_q1,_r2,_p2)>0.0f)
        {
            coord pp=getVector(_p2,_p1);
          float z1=dot(pp,n1);
          float z2=dot(v3,n1);
          float z3=dot(v4,n1);
          float lam1=-z1/z2;
          float lam2=-z1/z3;
          coord s1=obtainVector(_p2,lam1,v3);
          coord s2=obtainVector(_p2,lam2,v4);
          cout<<s1.x<<" "<<s1.y<<" "<<s1.z<<"\n";
          cout<<s2.x<<" "<<s2.y<<" "<<s2.z<<"\n";
          cout<<"i k l j\n";
        }
        else
        {
            coord pp=getVector(_p2,_p1);
          float z1=dot(pp,n1);
          float z2=dot(v3,n1);
          float lam1=-z1/z2;
          coord s1=obtainVector(_p2,lam1,v3);
          cout<<s1.x<<" "<<s1.y<<" "<<s1.z<<"\n";
          coord ss=getVector(_p1,_p2);
          z1=dot(ss,n2);
          z2=dot(v1,n2);
          float lam3=-z1/z2;
          coord s3=obtainVector(_p1,lam3,v1);
          cout<<s3.x<<" "<<s3.y<<" "<<s3.z<<"\n";
          cout<<"i k j l\n";
        }
      }
}

void triangleCollision :: checkCrossIntersection(coord _p1,coord _q1,coord _r1,coord _p2,coord _q2,coord _r2,float z1,float z2,float z3)
{
 	float zz1,zz2,zz3;
    zz1=determinant3d(_p1,_q1,_r1,_p2);
    zz2=determinant3d(_p1,_q1,_r1,_q2);
    zz3=determinant3d(_p1,_q1,_r1,_r2);
    int ff=2;
    if((zz1>0.0f && zz2>0.0f && zz3>0.0f) || (zz1<0.0f && zz2<0.0f && zz3<0.0f))
    ff=0;
    if(ff==0)
    {
      cout<<"No Intersection!";
      return;
    }

   	settingPositionsaccordingly(_p1,_q1,_r1,_p2,_q2,_r2,z1,z2,z3,zz1,zz2,zz3);
   	float pr1,pr2;
    pr1=determinant3d(_p1,_q1,_p2,_q2);
    pr2=determinant3d(_p1,_r1,_r2,_p2);
    if(pr1<=0.0f && pr2<=0.0f)
    {
    	cout<<"Triangles do intersect!\n";
        cout<<"Intersection :\n";
        coord v1=getVector(_q1,_p1);
        coord v2=getVector(_r1,_p1);
        coord v3=getVector(_q2,_p2);
  		coord v4=getVector(_r2,_p2);
        coord n1=cross(v1,v2);
        coord n2=cross(v3,v4);
        find3DTypesAndIntersections(_p1,_q1,_r1,_p2,_q2,_r2,v1,v2,v3,v4,n1,n2);// 3D
	}
	else
	cout<<"No intersection in Crossing\n";
}

void triangleCollision :: CoplanarDetectedRegion1(coord _p1,coord _q1,coord _r1,coord _p2,coord _q2,coord _r2,float d1,float d2,float d3)
{
//	cout<<d1<<" "<<d2<<" "<<d3<<"\n";

    if(d1>d2 && d1>d3)
    {
      coord s=_p2;
      _p2=_r2;
      _r2=_q2;
      _q2=s;
    }
    else if(d3>d1 && d3>d2)
    {
      coord s=_r2;
      _r2=_p2;
      _p2=_q2;
      _q2=s;
    }
    // assumption of _p1 to be in region R1:
    if(checkOrientation(_r2,_p2,_q1)>=0.0f)
    {
      if(checkOrientation(_r2,_p1,_q1)>=0.0f)
      {
        if(checkOrientation(_p1,_p2,_q1)>=0.0f)
        {
        cout<<"Intersection in 2D\n";
         findIntersection(_p1,_q1,_r1,_p2,_q2,_r2);

        }
        else
        {
          if(checkOrientation(_p1,_p2,_r1)>=0.0f)
          {
            if(checkOrientation(_q1,_r1,_p2)>=0.0f)
            {
            cout<<"Intersection in 2D\n";
             findIntersection(_p1,_q1,_r1,_p2,_q2,_r2);

            }
            else
            cout<<"No Intersection in 2D\n";
          }
          else
          cout<<"No Intersection in 2D\n";
        }
      }
      else
      cout<<"No Intersection in 2D\n";
    }
    else
    {
        if(checkOrientation(_r2,_p2,_r1)>=0.0f)
        {
          if(checkOrientation(_q1,_r1,_r2)>=0.0f)
          {
            if(checkOrientation(_p1,_p2,_r1)>0.0f)
            cout<<"No Intersection in 2D\n";
            else
             {
            cout<<"Intersection in 2D\n";
             findIntersection(_p1,_q1,_r1,_p2,_q2,_r2);

            }
          }
          else
          cout<<"No Intersection in 2D\n";
        }
        else
        cout<<"No Intersection in 2D\n";
    }
}

void triangleCollision :: CoplanarDetectedRegion2(coord _p1,coord _q1,coord _r1,coord _p2,coord _q2,coord _r2,float d1,float d2,float d3)
{
	// assumption of _p1 to be in region R2:
    if(d1<d2 && d1<d3)
    {
      coord s=_p2;
      _p2=_r2;
      _r2=_q2;
      _q2=s;
    }
    if(d2<d3 && d2<d1)
    {
      coord s=_q2;
      _q2=_r2;
      _r2=_p2;
      _p2=s;
    }
    if(checkOrientation(_r2,_p2,_q1)>=0.0f)
    {
      if(checkOrientation(_q2,_r2,_q1)>=0.0f)
      {
        if(checkOrientation(_p1,_p2,_q1)>=0.0f)
        {
          if(checkOrientation(_p1,_q2,_q1)>0.0f)
          cout<<"No Intersection in 2D.\n";
          else
           {
            cout<<"Intersection in 2D\n";
             findIntersection(_p1,_q1,_r1,_p2,_q2,_r2);

            }
        }
        else
        {
          if(checkOrientation(_p1,_p2,_r1)>=0.0f)
          {
            if(checkOrientation(_r2,_p2,_r1)>=0.0f)
             {
            cout<<"Intersection in 2D\n";
             findIntersection(_p1,_q1,_r1,_p2,_q2,_r2);

            }
            else
            {
              show(_p1);
              cout<<" \n";
            show(_p2);
            show(_q2);
            show(_r2);
            cout<<"No Intersection in 2D..\n";
            }
          }
          else
          cout<<"No Intersection in 2D...\n";
        }
      }
      else
      {
        if(checkOrientation(_p1,_q2,_q1)>0.0f)
        cout<<"No Intersection in 2D....\n";
        else
        {
          if(checkOrientation(_q2,_r2,_r1)>=0.0f)
          {
            if(checkOrientation(_q1,_r1,_q2)>=0.0f)
             {
            cout<<"Intersection in 2D\n";
             findIntersection(_p1,_q1,_r1,_p2,_q2,_r2);

            }
            else
            cout<<"No Intersection in 2D*\n";
          }
          else
          cout<<"No Intersection in 2D**\n";
        }
      }
    }
    else
    {
      if(checkOrientation(_r2,_p2,_r1)>=0.0f)
      {
        if(checkOrientation(_q1,_r1,_r2)>=0.0f)
        {
          if(checkOrientation(_r1,_p1,_p2)>=0.0f)
           {
            cout<<"Intersection in 2D\n";
             findIntersection(_p1,_q1,_r1,_p2,_q2,_r2);

            }
          else
          cout<<"No Intersection in 2D***\n";
        }
        else
        {
          if(checkOrientation(_q1,_r1,_q2)>=0.0f)
          {
            if(checkOrientation(_q2,_r2,_r1)>=0.0f)
             {
            cout<<"Intersection in 2D\n";
             findIntersection(_p1,_q1,_r1,_p2,_q2,_r2);

            }
            else
            cout<<"No Intersection in 2D****\n";
          }
          else
          cout<<"No Intersection in 2D#\n";
        }
      }
      else
      cout<<"No Intersection in 2D##\n";
    }
}

void triangleCollision :: checkCoplanarIntersection(coord _p1,coord _q1,coord _r1,coord _p2,coord _q2,coord _r2)
{
  	 //2-d test
    //ensuring counter-clockwise direction for T1
    if(checkOrientation(_p1,_q1,_r1)<0.0f)
   	swapPosition(&_q1,&_r1);
    //ensuring counter-clockwise direction for T2
    if(checkOrientation(_p2,_q2,_r2)<0.0f)
    swapPosition(&_q2,&_r2);
    bool y,z;
    bool intersection=false;
    if(checkingPointinTriangle(_p2,_q2,_r2,_p1))
    {
      cout<<"_p1 inside t2 in 2d plane\n";
      intersection=true;
    }
    float a1,a2,a3;
    a1=checkOrientation(_p1,_p2,_q2);
    a2=checkOrientation(_p1,_q2,_r2);
    a3=checkOrientation(_p1,_r2,_p2);
   // cout<<a1<<"--- "<<a2<<" "<<a3<<"\n";
    if(a1==0.0f || a2==0.0f || a3==0.0f)
    {
      if(a1==0.0f && a2==0.0f)
      cout<<"_p1 lies on vertex of t2\n";
      else if(a2==0.0f || a3==0.0f)
      cout<<"_p1 lies on vertex of t2\n";
      else if(a1==0.0f && a3==0.0f)
      cout<<"_p1 lies on vertex of t2\n";
      else
      cout<<"_p1 lies on an edge of t2\n";
      intersection=true;
    }
    if(intersection)
    {
      findIntersection(_p1,_q1,_r1,_p2,_q2,_r2);
      return;
    }
    int fz=0;
    if(a1>0.0f && a2>0.0f && a3<0.0f)
    fz=1;
    else if(a1>0.0f && a3>0.0f && a2<0.0f)
    fz=1;
    else if(a2>0.0f && a3>0.0f && a1<0.0f)
    fz=1;
    float d1,d2,d3;
    d1=getDistance(_p1,_p2);
    d2=getDistance(_p1,_q2);
    d3=getDistance(_p1,_r2);
    // need to ensure _p1 is distant from _q2 than _r2 and _p2 for region R1
    if(fz==1)
	CoplanarDetectedRegion1(_p1,_q1,_r1,_p2,_q2,_r2,d1,d2,d3);
	else
	CoplanarDetectedRegion2(_p1,_q1,_r1,_p2,_q2,_r2,d1,d2,d3);
}

int main()
{
	clock_t t;
	t = clock();
   triangleCollision object;
   object.take_input();
   object.check_type_of_intersection();
  t = clock() - t;
	cout << "time: " << t << " miliseconds" << "\n";
}
