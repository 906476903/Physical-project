#include <iostream>
#include <SDL/SDL.h>
#include <SDL/SDL_image.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <Box2D/Box2D.h>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <ctime>
#include "Physics/Physics.h"
#include "Physics/Const.h"
#include "Physics/Light.h"

using namespace std;

vector<b2Vec2> gpoint;
Simulator sb;
light light_point[10];
Uint32 start, gameon;
int light_num;
bool collision_flag;

const int MAXLEVEL = 8;
void (*init_func[MAXLEVEL])(int&, int&);
unsigned now_time;
class colorVector
{
public:
	double r, g, b;
	colorVector(){}
	colorVector(double r, double g, double b): r(r), g(g), b(b){}
	double len()
	{
		return sqrt(r * r + g * g + b * b);
	}
	colorVector operator - (const colorVector &x) const
	{
		return colorVector(r - x.r, g - x.g, b - x.b);
	}
	colorVector operator * (const double x) const
	{
		return colorVector(r * x, g * x, b * x);
	}
	colorVector operator / (const double x) const
	{
		return colorVector(r / x, g / x, b / x);
	}
	colorVector operator + (const colorVector &x) const
	{
		return colorVector(r + x.r, g + x.g, b + x.b);
	}
};
colorVector color[3];

void change_color(int size = 0)
{
	static double current_length = 0;
	static int start = 0;
	if(current_length > (color[(start + 1) % 3] - color[start]).len())
		start = (start + 1) % 3, current_length = 0;
	if (now_time != time(0))
	{
		current_length += 0.00005;
	}
	colorVector dir = color[(start + 1) % 3] - color[start];
	dir = dir / dir.len();
	dir = dir * current_length;
	dir = dir + color[start];
	if (size > 300)
		glColor3f(1, 0, 0);
	else
		glColor3f(dir.r, dir.g, dir.b);
}

b2Body* addRect(int x,int y,int w,int h,bool dyn=true)
{
	b2BodyDef bodydef;
	bodydef.position.Set(x*P2M,y*P2M);
	if(dyn)
		bodydef.type=b2_dynamicBody;
	bodydef.linearDamping = 0.0f;
	bodydef.angularDamping = 0.0f;

	b2Body* body=sb.world->CreateBody(&bodydef);

	b2PolygonShape shape;
	shape.SetAsBox(P2M*w/2,P2M*h/2);

	b2FixtureDef fixturedef;
	fixturedef.shape=&shape;
	fixturedef.density=1.0;
	fixturedef.restitution = 1.0;
	fixturedef.friction = 0;
	body->CreateFixture(&fixturedef);
	return NULL;
}

void drawSquare(b2Vec2* points,b2Vec2 center,float angle)
{
	glColor3f(1,1,1);
	glPushMatrix();
		glTranslatef(center.x*M2P,center.y*M2P,0);

		glRotatef(angle*180.0/M_PI,0,0,1);
		glBegin(GL_QUADS);
			for(int i=0;i<4;i++)
				glVertex2f(points[i] .x*M2P,points[i] .y*M2P);
		glEnd();
	glPopMatrix();
}

void draw_poly(vector<b2Vec2> points,b2Vec2 center,float angle)
{
	change_color(points.size());
	glPushMatrix();
		glTranslatef(center.x*M2P,center.y*M2P,0);

		glRotatef(angle*180.0/M_PI,0,0,1);
		glBegin(GL_POLYGON);
			for(int i=0;i!=points.size();++i)
				glVertex2f(points[i] .x*M2P,points[i] .y*M2P);
		glEnd();
	glPopMatrix();
}
void change_color_mirror()
{
        glColor3f(0, 0, 0);
        glColor3f(0, 0, 0.9);
}

void draw_mirror_poly(vector<b2Vec2> points,b2Vec2 center,float angle)
{
	change_color_mirror();
	glPushMatrix();
		glTranslatef(center.x*M2P,center.y*M2P,0);

		glRotatef(angle*180.0/M_PI,0,0,1);
		glBegin(GL_POLYGON);
			for(int i=0;i!=points.size();++i)
				glVertex2f(points[i] .x*M2P,points[i] .y*M2P);
		glEnd();
	glPopMatrix();
}

void display(int dx,int dy)
{
	glClear(GL_COLOR_BUFFER_BIT);
	glLoadIdentity();
	#ifdef __APPLE__
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_POLYGON_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	#endif
	b2Body* tmp=sb.world->GetBodyList();
	vector<b2Vec2> points;
	int count = 0;
  while(tmp) {
          count++;
          int size = ((b2PolygonShape*)((tmp -> GetFixtureList()[0]).GetShape())) -> GetVertexCount();
          points.resize(size);
          for(int i=0;i<size;i++)
                  points[i] =((b2PolygonShape*)tmp->GetFixtureList()->GetShape())->GetVertex(i);
          
          if(size == 4 && fabs(points[0].y - points[3].y) < 10.0/18 ) 
                  draw_mirror_poly(points,tmp->GetWorldCenter(),tmp->GetAngle());//画镜子
          else
                  draw_poly(points,tmp->GetWorldCenter(),tmp->GetAngle());
          tmp=tmp->GetNext();
  }

  /*
  cerr << "first block" << " " << ((SDL_GetTicks()-gameon) / 1000.0) << " " << start << " " << (int)((SDL_GetTicks()-gameon) / 1000.0) << endl;
  */
  if((int)((SDL_GetTicks()-gameon) / 1000.0) % 2 == 0) {
	//画光线
          //cerr << "second block" << endl;
          for (int w = 0; w < light_num; w += 2) {
                  glColor3f(255,255,0);//黄色
                  glPushMatrix();
                  glBegin(GL_LINE_STRIP);
                  vector<b2Vec2> seg_point = light_point[w].get_light();
                  //cerr << seg_point.size() << endl;
                  for (int i = 0; i < seg_point.size(); i++)
                          cerr << seg_point[i].x << " " << seg_point[i].y << endl;
                  for (int i = 0; i < (int)seg_point.size(); i ++)
                          glVertex2f(seg_point[i].x, seg_point[i].y);
                  glEnd();
                  glPopMatrix();
                  if (sb.kill(seg_point)) {
                          // for collision
                          std::cerr << "Collision" << std::endl;
                          collision_flag = true;
                          return ;
                  }
          }

  }

  if((int)((SDL_GetTicks()-gameon) / 1000.0) % 2 == 1) {
          //画光线
          //cerr << "second block" << endl;
          for (int w = 1; w < light_num; w += 2) {
                  glColor3f(255,255,0);//黄色
                  glPushMatrix();
                  glBegin(GL_LINE_STRIP);
                  vector<b2Vec2> seg_point = light_point[w].get_light();
                  //cerr << seg_point.size() << endl;
                  for (int i = 0; i < seg_point.size(); i++)
                          cerr << seg_point[i].x << " " << seg_point[i].y << endl;
                  for (int i = 0; i < (int)seg_point.size(); i++)
                          glVertex2f(seg_point[i].x, seg_point[i].y);
                  glEnd();
                  glPopMatrix();
                  if (sb.kill(seg_point)) {
                          // for collision
                          std::cerr << "Collision" << std::endl;
                          collision_flag = true;
                          return ;
                  }
          }
  }

	//画目的地
	glColor3f(0,1,0);
	glPushMatrix();
	glBegin(GL_POLYGON);
	glVertex2f(dx-3,dy-3);	
	glVertex2f(dx-3,dy+3);
	glVertex2f(dx+3,dy+3);
	glVertex2f(dx+3,dy-3);
	glEnd();
	glPopMatrix();

	//画death地
	glColor3f(1,0,0);
	for(int i=0; i!=sb.deathPoint.size();++i)
	{
		glPushMatrix();
		glBegin(GL_POLYGON);
		glVertex2f(sb.deathPoint[i].x * M2P-3,sb.deathPoint[i].y * M2P-3);	
		glVertex2f(sb.deathPoint[i].x * M2P-3,sb.deathPoint[i].y * M2P+3);
		glVertex2f(sb.deathPoint[i].x * M2P+3,sb.deathPoint[i].y * M2P+3);
		glVertex2f(sb.deathPoint[i].x * M2P+3,sb.deathPoint[i].y * M2P-3);
		glEnd();
		glPopMatrix();
	}
	glColor3f(1,1,1);
	glPushMatrix();
	glBegin(GL_LINE_STRIP);
	for(int i=0;i!=gpoint.size();++i)
			glVertex2f(gpoint[i].x,gpoint[i].y);
	glEnd();
	glPopMatrix();
}

void moveRect(double x, double y)
{
	b2Body* tmp = sb.world -> GetBodyList();
	b2Vec2 points[4];
	while(tmp)
	{
		int fac1 = 1, fac2 = 1;
		if(rand() % 2) fac1 *= -1;
		if(rand() % 2) fac2 *= -1;
		tmp -> SetLinearVelocity(b2Vec2(rand() % 10  * fac1, rand() % 10  * fac2));
		tmp = tmp -> GetNext();
	}
}

void init_level0(int &dx, int &dy)
{
	g = 9.8;
	int centerx,centery;//开始点坐标
  light_num = 0;

	centerx = 100; centery = HEIGHT/2 + 80;
	dx=WIDTH - 100; dy=HEIGHT/2; 
	vector< b2Vec2 > death_point;
	
	{//设置障碍物
		vector<pair<vector<b2Vec2>, bool> > GameMap;
		vector<b2Vec2> goods;
		vector<vector<b2Vec2> > backup_goods,backup_mirror;
		vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field;
		//GameMap.push_back( pair< b2Vec2(,) , false> );
		goods.push_back( b2Vec2(0,0) );
		goods.push_back( b2Vec2(0,10) );
		goods.push_back( b2Vec2(WIDTH,10) );
		goods.push_back( b2Vec2(WIDTH,0) );
		//GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//初始世界中的墙
		goods.clear();
		
		
		goods.push_back( b2Vec2(WIDTH,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,HEIGHT-10) );
		goods.push_back( b2Vec2(0,HEIGHT-10) );
		goods.push_back( b2Vec2(0,HEIGHT) );
		//GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//初始世界中的墙
		goods.clear();
		
		goods.push_back( b2Vec2(0,0) );
		goods.push_back( b2Vec2(0,HEIGHT) );
		goods.push_back( b2Vec2(10,HEIGHT) );
		goods.push_back( b2Vec2(10,0) );
		//GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//初始世界中的墙
		goods.clear();
		
		goods.push_back( b2Vec2(WIDTH-10,0) );
		goods.push_back( b2Vec2(WIDTH-10,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,0) );
		//GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//初始世界中的墙
		goods.clear();
		
		goods.push_back( b2Vec2(0,0) );
		goods.push_back( b2Vec2(0,HEIGHT/3) );
		goods.push_back( b2Vec2(WIDTH,HEIGHT/3) );
		goods.push_back( b2Vec2(WIDTH,0) );
		GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//将世界中的墙（除了初始化的四面墙和激光发生器之外）备份
		goods.clear();
		

		goods.push_back( b2Vec2(0,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,2 * HEIGHT/3) );
		goods.push_back( b2Vec2(0,2 * HEIGHT/3) );
		GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);
		goods.clear();
		

		//激光发生器
		goods.push_back( b2Vec2(30 - 20,2*HEIGHT/3 - 9) );
		goods.push_back( b2Vec2(30 - 20,2*HEIGHT/3 - 9 - 20) );
		goods.push_back( b2Vec2(50 - 20,2*HEIGHT/3 - 9));
		goods.push_back( b2Vec2(50 - 20,2*HEIGHT/3 - 9 - 20) );
		GameMap.push_back(make_pair(goods, false));
		goods.clear();


		//以下添加物体为镜子
		//如果一个多边形的点数只有4,而且高度大于5小于10,则这个物体为镜子并且打印为淡蓝色(为了打印时区分).
		goods.push_back( b2Vec2(0,2*HEIGHT/3 - 9) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(0,2*HEIGHT/3) );
		goods.push_back( b2Vec2(WIDTH,2*HEIGHT/3) );
		goods.push_back( b2Vec2(WIDTH,2*HEIGHT/3 - 9) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

		goods.push_back( b2Vec2(0,HEIGHT/3 + 9) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(0,HEIGHT/3) );
		goods.push_back( b2Vec2(WIDTH,HEIGHT/3) );
		goods.push_back( b2Vec2(WIDTH,HEIGHT/3 + 9) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

		light_point[light_num++].construct( b2Vec2(20,2*HEIGHT/3 - 9 - 10), b2Vec2(10,-10), backup_goods, backup_mirror);
    light_point[light_num++].construct(b2Vec2(30, 340), b2Vec2(10, -1), backup_goods, backup_mirror);
		sb.create(GameMap , field, death_point , 0.5, false);
	}
	sb.set_goal(dx,dy);

	for(int i = 0; i < 360; ++i) {gpoint.push_back(b2Vec2(centerx + cos((double)i*M_PI/180) * 30, centery + sin((double)i*M_PI/180)*30));}
	sb.addPolygon(gpoint, true, 0, 0, -1); gpoint.clear();
}
void init_level1(int &dx, int &dy)
{
	g = 9.8;
	int centerx,centery;//开始点坐标
  light_num = 0;
	centerx = 100; centery = HEIGHT/2 - 100;
	dx=WIDTH - 100; dy=HEIGHT/2;
	vector< b2Vec2 > death_point;

	{//设置障碍物
		vector<pair<vector<b2Vec2>, bool> > GameMap;
		vector<b2Vec2> goods;
		vector<vector<b2Vec2> > backup_goods,backup_mirror;
		vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field;
		//GameMap.push_back( pair< b2Vec2(,) , false> );

    goods.push_back( b2Vec2(0,0) );
		goods.push_back( b2Vec2(0,10) );
		goods.push_back( b2Vec2(WIDTH,10) );
		goods.push_back( b2Vec2(WIDTH,0) );
		//GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//初始世界中的墙
		goods.clear();

		goods.push_back( b2Vec2(WIDTH,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,HEIGHT-10) );
		goods.push_back( b2Vec2(0,HEIGHT-10) );
		goods.push_back( b2Vec2(0,HEIGHT) );
		//GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//初始世界中的墙
		goods.clear();

		goods.push_back( b2Vec2(0,0) );
		goods.push_back( b2Vec2(0,HEIGHT) );
		goods.push_back( b2Vec2(10,HEIGHT) );
		goods.push_back( b2Vec2(10,0) );
		//GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//初始世界中的墙
		goods.clear();

		goods.push_back( b2Vec2(WIDTH-10,0) );
		goods.push_back( b2Vec2(WIDTH-10,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,0) );
		//GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//初始世界中的墙
		goods.clear();

		goods.push_back( b2Vec2(200, HEIGHT) );
		goods.push_back( b2Vec2(300, HEIGHT) );
		goods.push_back( b2Vec2(300, HEIGHT - 200) );
		goods.push_back( b2Vec2(200, HEIGHT - 200) );
		GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//将世界中的墙（除了初始化的四面墙和激光发生器之外）备份
		goods.clear();

		goods.push_back( b2Vec2(400, HEIGHT) );
		goods.push_back( b2Vec2(600, HEIGHT) );
		goods.push_back( b2Vec2(600, HEIGHT - 150) );
		goods.push_back( b2Vec2(400, HEIGHT - 150) );
		GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//将世界中的墙（除了初始化的四面墙和激光发生器之外）备份
		goods.clear();

    goods.push_back( b2Vec2(700, HEIGHT) );
		goods.push_back( b2Vec2(800, HEIGHT) );
		goods.push_back( b2Vec2(800, HEIGHT - 250) );
		goods.push_back( b2Vec2(700, HEIGHT - 250) );
		GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//将世界中的墙（除了初始化的四面墙和激光发生器之外）备份
		goods.clear();

    goods.push_back( b2Vec2(900, HEIGHT) );
		goods.push_back( b2Vec2(1000, HEIGHT) );
		goods.push_back( b2Vec2(1000, HEIGHT - 350) );
		goods.push_back( b2Vec2(900, HEIGHT - 350) );
		GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//将世界中的墙（除了初始化的四面墙和激光发生器之外）备份
		goods.clear();

		//激光发生器
		goods.push_back( b2Vec2(10, 350) );
		goods.push_back( b2Vec2(30, 350) );
		goods.push_back( b2Vec2(30, 330) );
		goods.push_back( b2Vec2(10, 330) );
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

		//以下添加物体为镜子
		//如果一个多边形的点数只有4,而且高度大于5小于10,则这个物体为镜子并且打印为淡蓝色(为了打印时区分).

		goods.push_back( b2Vec2(1260, 219) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(1270, 219) );
		goods.push_back( b2Vec2(1270, 213) );
		goods.push_back( b2Vec2(1260, 213) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

    goods.push_back( b2Vec2(10, 95) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(20, 95) );
		goods.push_back( b2Vec2(20, 89) );
		goods.push_back( b2Vec2(10, 89) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
		goods.clear();
    /*
		goods.push_back( b2Vec2(0,2 * HEIGHT/3 - 9) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(0,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,2 * HEIGHT/3 - 9) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
		goods.clear();
    */
		light_point[light_num++].construct(b2Vec2(30, 340), b2Vec2(10, -1), backup_goods, backup_mirror);
		sb.create(GameMap , field, death_point , 0.5, false);
	}
	sb.set_goal(dx,dy);

	for(int i = 0; i < 360; ++i) {gpoint.push_back(b2Vec2(centerx + cos((double)i*M_PI/180) * 30, centery + sin((double)i*M_PI/180)*30));}
	sb.addPolygon(gpoint, true, 0, 0, -1); gpoint.clear();
}

void init_level2(int &dx, int &dy)
{
        g = 9.8;
        int centerx, centery;
        light_num = 0;

        centerx = 50; centery = HEIGHT / 2 - 330;
        dx = WIDTH - 100; dy = HEIGHT / 2;
        vector<b2Vec2> death_point;
	{//设置障碍物
		vector<pair<vector<b2Vec2>, bool> > GameMap;
		vector<b2Vec2> goods;
		vector<vector<b2Vec2> > backup_goods,backup_mirror;
		vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field;
		//GameMap.push_back( pair< b2Vec2(,) , false> );
		goods.push_back( b2Vec2(0,0) );
		goods.push_back( b2Vec2(0,10) );
		goods.push_back( b2Vec2(WIDTH,10) );
		goods.push_back( b2Vec2(WIDTH,0) );
		//GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//初始世界中的墙
		goods.clear();

		goods.push_back( b2Vec2(WIDTH,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,HEIGHT-10) );
		goods.push_back( b2Vec2(0,HEIGHT-10) );
		goods.push_back( b2Vec2(0,HEIGHT) );
		//GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//初始世界中的墙
		goods.clear();

		goods.push_back( b2Vec2(0,0) );
		goods.push_back( b2Vec2(0,HEIGHT) );
		goods.push_back( b2Vec2(10,HEIGHT) );
		goods.push_back( b2Vec2(10,0) );
		//GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//初始世界中的墙
		goods.clear();

		goods.push_back( b2Vec2(WIDTH-10,0) );
		goods.push_back( b2Vec2(WIDTH-10,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,0) );
		//GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//初始世界中的墙
		goods.clear();


    //障碍物
		goods.push_back( b2Vec2(10, 100) );
		goods.push_back( b2Vec2(80, 100) );
		goods.push_back( b2Vec2(80, 80) );
		goods.push_back( b2Vec2(10, 80) );
		GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);
		goods.clear();

		goods.push_back( b2Vec2(600, 500) );
		goods.push_back( b2Vec2(640, 500) );
		goods.push_back( b2Vec2(640, 200) );
		goods.push_back( b2Vec2(600, 200) );
		GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);
		goods.clear();

    goods.push_back( b2Vec2(260, 450) );
		goods.push_back( b2Vec2(400, 450) );
		goods.push_back( b2Vec2(400, 430) );
		goods.push_back( b2Vec2(260, 430) );
		GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);
		goods.clear();

		//激光发生器
		goods.push_back( b2Vec2(200, 10) );
		goods.push_back( b2Vec2(200, 30) );
		goods.push_back( b2Vec2(230, 30));
		goods.push_back( b2Vec2(230, 10) );
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

    goods.push_back( b2Vec2(400, 440) );
		goods.push_back( b2Vec2(400, 450) );
		goods.push_back( b2Vec2(420, 450));
		goods.push_back( b2Vec2(420, 440) );
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

    goods.push_back( b2Vec2(640, 420) );
		goods.push_back( b2Vec2(640, 430) );
		goods.push_back( b2Vec2(730, 430));
		goods.push_back( b2Vec2(730, 420) );
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

    goods.push_back( b2Vec2(880, 10) );
		goods.push_back( b2Vec2(880, 20) );
		goods.push_back( b2Vec2(900, 20));
		goods.push_back( b2Vec2(900, 10) );
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

		//以下添加物体为镜子
		//如果一个多边形的点数只有4,而且高度大于5小于10,则这个物体为镜子并且打印为淡蓝色(为了打印时区分).
    goods.push_back( b2Vec2(1250, 135) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(1270, 135) );
		goods.push_back( b2Vec2(1270, 145) );
		goods.push_back( b2Vec2(1250, 145) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
    goods.clear();

    goods.push_back( b2Vec2(580, 390) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(600, 390) );
		goods.push_back( b2Vec2(600, 400) );
		goods.push_back( b2Vec2(580, 400) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
    goods.clear();

    goods.push_back( b2Vec2(250, 670) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(400, 670) );
		goods.push_back( b2Vec2(400, 660) );
		goods.push_back( b2Vec2(250, 660) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
    goods.clear();

    goods.push_back( b2Vec2(40, 400) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(50, 400) );
		goods.push_back( b2Vec2(50, 390) );
		goods.push_back( b2Vec2(40, 390) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
    goods.clear();

    goods.push_back( b2Vec2(420, 20) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(440, 20) );
		goods.push_back( b2Vec2(440, 10) );
		goods.push_back( b2Vec2(420, 10) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
    goods.clear();

    goods.push_back( b2Vec2(590, 210) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(610, 210) );
		goods.push_back( b2Vec2(610, 200) );
		goods.push_back( b2Vec2(590, 200) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
    goods.clear();

    light_point[light_num++].construct(b2Vec2(215, 30), b2Vec2(2, 2), backup_goods, backup_mirror);
    light_point[light_num++].construct(b2Vec2(420, 450), b2Vec2(2, 2), backup_goods, backup_mirror);
    light_point[light_num++].construct(b2Vec2(890, 20), b2Vec2(0, 2), backup_goods, backup_mirror);
    light_point[light_num++].construct(b2Vec2(730, 425), b2Vec2(2, 0), backup_goods, backup_mirror);
    //light_point[light_num++].construct(b2Vec2(560, 30), b2Vec2(10, 10), backup_goods, backup_mirror);
		sb.create(GameMap , field, death_point , 0.5, false);
	}
	sb.set_goal(dx,dy);

	for(int i = 0; i < 360; ++i) {gpoint.push_back(b2Vec2(centerx + cos((double)i*M_PI/180) * 30, centery + sin((double)i*M_PI/180)*30));}
	sb.addPolygon(gpoint, true, 0, 0, -1); gpoint.clear();
}

void init_level3(int &dx, int &dy)
{
	g = 9.8;
	int centerx,centery;//开始点坐标
  light_num = 0;

	centerx = 100; centery = HEIGHT/2 - 300;
	dx=WIDTH - 100; dy=HEIGHT/2;
	vector< b2Vec2 > death_point;

	{//设置障碍物
		vector<pair<vector<b2Vec2>, bool> > GameMap;
		vector<b2Vec2> goods;
		vector<vector<b2Vec2> > backup_goods,backup_mirror;
		vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field;
		//GameMap.push_back( pair< b2Vec2(,) , false> );
		goods.push_back( b2Vec2(0,0) );
		goods.push_back( b2Vec2(0,10) );
		goods.push_back( b2Vec2(WIDTH,10) );
		goods.push_back( b2Vec2(WIDTH,0) );
		//GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//初始世界中的墙
		goods.clear();

		goods.push_back( b2Vec2(WIDTH,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,HEIGHT-10) );
		goods.push_back( b2Vec2(0,HEIGHT-10) );
		goods.push_back( b2Vec2(0,HEIGHT) );
		//GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//初始世界中的墙
		goods.clear();

		goods.push_back( b2Vec2(0,0) );
		goods.push_back( b2Vec2(0,HEIGHT) );
		goods.push_back( b2Vec2(10,HEIGHT) );
		goods.push_back( b2Vec2(10,0) );
		//GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//初始世界中的墙
		goods.clear();

		goods.push_back( b2Vec2(WIDTH-10,0) );
		goods.push_back( b2Vec2(WIDTH-10,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,0) );
		//GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);//初始世界中的墙
		goods.clear();


    //障碍物
		goods.push_back( b2Vec2(500, 380) );
		goods.push_back( b2Vec2(550, 380) );
		goods.push_back( b2Vec2(550, 10) );
		goods.push_back( b2Vec2(500, 10) );
		GameMap.push_back(make_pair(goods, false));
		backup_goods.push_back(goods);
		goods.clear();

		//激光发生器
		goods.push_back( b2Vec2(10, 300) );
		goods.push_back( b2Vec2(10, 320) );
		goods.push_back( b2Vec2(30, 320));
		goods.push_back( b2Vec2(30, 300) );
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

		goods.push_back( b2Vec2(10, 680) );
		goods.push_back( b2Vec2(10, 700) );
		goods.push_back( b2Vec2(30, 700));
		goods.push_back( b2Vec2(30, 680) );
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

    goods.push_back( b2Vec2(550, 10) );
		goods.push_back( b2Vec2(570, 30) );
		goods.push_back( b2Vec2(550, 30));
		goods.push_back( b2Vec2(570, 10) );
		GameMap.push_back(make_pair(goods, false));
		goods.clear();
		//以下添加物体为镜子
		//如果一个多边形的点数只有4,而且高度大于5小于10,则这个物体为镜子并且打印为淡蓝色(为了打印时区分).
    goods.push_back( b2Vec2(1250, 135) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(1270, 135) );
		goods.push_back( b2Vec2(1270, 145) );
		goods.push_back( b2Vec2(1250, 145) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
    goods.clear();

    goods.push_back( b2Vec2(900, 580) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(1050, 580) );
		goods.push_back( b2Vec2(1050, 590) );
		goods.push_back( b2Vec2(900, 590) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
    goods.clear();

		goods.push_back( b2Vec2(150, 150) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(400, 150) );
		goods.push_back( b2Vec2(400, 160) );
		goods.push_back( b2Vec2(150, 160) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

    goods.push_back( b2Vec2(150, 550) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(400, 550) );
		goods.push_back( b2Vec2(400, 560) );
		goods.push_back( b2Vec2(150, 560) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

    goods.push_back( b2Vec2(700, 300) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(950, 300) );
		goods.push_back( b2Vec2(950, 310) );
		goods.push_back( b2Vec2(700, 310) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
    goods.clear();

    goods.push_back( b2Vec2(600, 430) );//这里将镜子的高度设为9
		goods.push_back( b2Vec2(950, 430) );
		goods.push_back( b2Vec2(950, 440) );
		goods.push_back( b2Vec2(600, 440) );//这里将镜子的高度设为9
		backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
    goods.clear();

    goods.push_back( b2Vec2(0,0) );
		goods.push_back( b2Vec2(0,10) );
		goods.push_back( b2Vec2(WIDTH,10) );
		goods.push_back( b2Vec2(WIDTH,0) );
    backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

		goods.push_back( b2Vec2(WIDTH,HEIGHT) );
		goods.push_back( b2Vec2(WIDTH,HEIGHT-10) );
		goods.push_back( b2Vec2(0,HEIGHT-10) );
		goods.push_back( b2Vec2(0,HEIGHT) );
    backup_mirror.push_back(goods);
		GameMap.push_back(make_pair(goods, false));
		goods.clear();

    light_point[light_num++].construct(b2Vec2(30, 305), b2Vec2(2, -3), backup_goods, backup_mirror);
    light_point[light_num++].construct(b2Vec2(20, 690), b2Vec2(10, -17), backup_goods, backup_mirror);
    light_point[light_num++].construct(b2Vec2(560, 30), b2Vec2(10, 10), backup_goods, backup_mirror);
		sb.create(GameMap , field, death_point , 0.5, false);
	}
	sb.set_goal(dx,dy);

	for(int i = 0; i < 360; ++i) {gpoint.push_back(b2Vec2(centerx + cos((double)i*M_PI/180) * 30, centery + sin((double)i*M_PI/180)*30));}
	sb.addPolygon(gpoint, true, 0, 0, -1); gpoint.clear();
}

void init_level4(int &dx, int &dy)
{
	light_num=0;
	vector<b2Vec2> gpoint;
	vector< b2Vec2 > death_point;
	vector<pair<vector<b2Vec2>, bool> > GameMap;
	vector<b2Vec2> goods;
	vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field;

	goods.push_back(b2Vec2(0, 0));
	goods.push_back(b2Vec2(0, HEIGHT));
	goods.push_back(b2Vec2(600, HEIGHT));
	goods.push_back(b2Vec2(600, 0));

	GameMap.push_back(make_pair(goods, false));

	goods.clear();

	goods.push_back(b2Vec2(600 + 400, 0));
	goods.push_back(b2Vec2(600 + 400, HEIGHT));
	goods.push_back(b2Vec2(600 + 600, HEIGHT));
	goods.push_back(b2Vec2(600 + 600, 0));

	GameMap.push_back(make_pair(goods, false));

	goods.clear();
	goods.push_back(b2Vec2(600, HEIGHT/2+1));
	goods.push_back(b2Vec2(600, HEIGHT/2));
	goods.push_back(b2Vec2(600 + 200, HEIGHT/2));
	goods.push_back(b2Vec2(600 + 200, HEIGHT/2+1));
	GameMap.push_back(make_pair(goods, false));

	goods.clear();
	goods.push_back(b2Vec2(600+240, HEIGHT/2+1));
	goods.push_back(b2Vec2(600+240, HEIGHT/2));
	goods.push_back(b2Vec2(600 + 400, HEIGHT/2));
	goods.push_back(b2Vec2(600 + 400, HEIGHT/2+1));
	GameMap.push_back(make_pair(goods, false));

	goods.clear();	
	goods.push_back(b2Vec2(600 + 400, 0));
	goods.push_back(b2Vec2(600 + 400, HEIGHT));
	goods.push_back(b2Vec2(600 + 600, HEIGHT));
	goods.push_back(b2Vec2(600 + 600, 0));

	GameMap.push_back(make_pair(goods, false));

	goods.clear();
	sb.create(GameMap, field, death_point , 0.5, false);

	double centerx = 100, centery = HEIGHT/2 - 100;
  	for(int i = 0; i < 1000; ++i)
  	{
  		centerx = rand() % 200 + 600, centery = rand() % 100;
  		for(int j = 0; j < 3; ++j) {gpoint.push_back(b2Vec2(centerx + cos((double)j*120*M_PI/180) * 5, centery + sin((double)j*120*M_PI/180)*5));}
  		sb.addPolygon(gpoint, true);
  		gpoint.clear();
  		cerr << i << endl;
  	}
}

void init_level5(int &dx, int &dy)
{
	light_num=0;
	vector<b2Vec2> gpoint;
	vector< b2Vec2 > death_point;
	vector<pair<vector<b2Vec2>, bool> > GameMap;
	vector<b2Vec2> goods;
	vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field;

	goods.push_back(b2Vec2(0, 0));
	goods.push_back(b2Vec2(0, HEIGHT));
	goods.push_back(b2Vec2(600, HEIGHT));
	goods.push_back(b2Vec2(600, 0));

	GameMap.push_back(make_pair(goods, false));

	goods.clear();

	goods.push_back(b2Vec2(600 + 400, 0));
	goods.push_back(b2Vec2(600 + 400, HEIGHT));
	goods.push_back(b2Vec2(600 + 600, HEIGHT));
	goods.push_back(b2Vec2(600 + 600, 0));

	GameMap.push_back(make_pair(goods, false));

	goods.clear();
	sb.create(GameMap, field, death_point , 0.5, false);

	double centerx = 100, centery = HEIGHT/2 - 100;
  	for(int i = 0; i < 2000; ++i)
  	{
  		centerx = rand() % 200 + 600, centery = rand() % 500;
  		for(int j = 0; j < 3; ++j) {gpoint.push_back(b2Vec2(centerx + cos((double)j*120*M_PI/180) * 5, centery + sin((double)j*120*M_PI/180)*5));}
  		sb.addPolygon(gpoint, true);
  		gpoint.clear();
  		cerr << i << endl;
  	}
}
void init_level6(int &dx, int &dy)
{
	light_num=0;
	vector<b2Vec2> gpoint;
	vector< b2Vec2 > death_point;
	vector<pair<vector<b2Vec2>, bool> > GameMap;
	vector<b2Vec2> goods;
	vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field;

  cout << "Block1" << endl;

	sb.create(GameMap, field, death_point , 0.5, false);
	double centerx = WIDTH/2, centery=30+300;
	for(int i = 0; i < 360; ++i) {gpoint.push_back(b2Vec2(centerx + cos((double)i*M_PI/180) * 15, centery + sin((double)i*M_PI/180)*15));}
	sb.addPolygon(gpoint, false, 0, 0, 100); gpoint.clear();

	centerx = WIDTH/2, centery=130+300;
	for(int i = 0; i < 360; ++i) {gpoint.push_back(b2Vec2(centerx + cos((double)i*M_PI/180) * 15, centery + sin((double)i*M_PI/180)*15));}
	sb.addPolygon(gpoint, true, 0, 0, 200); gpoint.clear();

  cout << "Block2" << endl;

	centerx = WIDTH/2+300, centery=130+300;
	for(int i = 0; i < 360; ++i) {gpoint.push_back(b2Vec2(centerx + cos((double)i*M_PI/180) * 15, centery + sin((double)i*M_PI/180)*15));}
	sb.addPolygon(gpoint, true, 0, 0, 300); gpoint.clear();

  cout << "Block3" << endl;

	const int k = 100;
	sb.relation.push_back(make_pair(make_pair(200, 100), make_pair(k, P2M * 100)));
	sb.relation.push_back(make_pair(make_pair(200, 300), make_pair(k, P2M * 100)));
}

void init_level7(int &dx, int &dy)
{
	int centerx,centery;//开始点坐标
	
	centerx = 0; centery = 0;
	dx=WIDTH+1000; dy=HEIGHT+1000;
	vector< b2Vec2 > death_point;
	vector<pair<vector<b2Vec2>, bool> > GameMap;
	vector<b2Vec2> goods;
	vector<pair<pair<b2Vec2, double>, pair<double, bool> > > field;
	srand(time(0));
	
		goods.push_back( b2Vec2(10,10) );
		goods.push_back( b2Vec2(10,70) );
		goods.push_back( b2Vec2(WIDTH-11,70) );
		goods.push_back( b2Vec2(WIDTH-11,10) );
		GameMap.push_back(make_pair(goods, true));
		goods.clear();
		
	for(int i = 0; i < 500; ++i)
	{
		field.push_back(make_pair(make_pair(b2Vec2(rand() % WIDTH, rand() % HEIGHT + 70), 0)
		, make_pair(2, true)));
	}
	g = 4.0;
	sb.create(GameMap , field, death_point , 0.5, true);
	sb.set_goal(dx,dy);

	b2Body* tmp = sb.world->GetBodyList();
	while(tmp)
	{
		int v = *((int*)tmp -> GetUserData());
		int fac1, fac2;
		fac1 = (rand() % 2) ? 1 : -1;
		fac2 = (rand() % 2) ? 1 : -1;
		if(v >= 10000)
		{
			tmp -> ApplyForce(b2Vec2((rand() * 2) % 200 * fac1, (rand() * 2) % 200 * fac2), tmp -> GetWorldCenter());
		}
		tmp = tmp -> GetNext();
	}
}

bool level(int &levelid)
{
	SDL_Event event;
	bool running=true;

	int dx,dy;//目的地
	if(levelid >= MAXLEVEL) return true;
	gpoint.clear();
	init_func[levelid](dx, dy);

  collision_flag = false;

	while(running)
	{
          if(collision_flag) {
                  collision_flag = false;
                  sb.destroy();
                  running=false;
                  continue;
          }
		start=SDL_GetTicks();
		while(SDL_PollEvent(&event))
		{
			switch(event.type)
			{
				case SDL_QUIT: { running=false; sb.destroy(); break;}

				case SDL_KEYDOWN:
					switch(event.key.keysym.sym)
					{
						case SDLK_ESCAPE: { running=false; sb.destroy(); exit(0); }
						case SDLK_UP:
						{
							sb.ApplyImpulseToMainRole(b2Vec2(0, -4));
							break;
						}
						case SDLK_DOWN:
						{

							sb.ApplyImpulseToMainRole(b2Vec2(0, 4));
							break;
						}
						case SDLK_RIGHT:
						{
							sb.ApplyImpulseToMainRole(b2Vec2(4, 0));
							break;
						}
						case SDLK_LEFT:
						{

							sb.ApplyImpulseToMainRole(b2Vec2(-4, 0));
							break;
						}
						case SDLK_SPACE:
						{
							sb.destroy();
							running=false;
							return false;
						}
						case SDLK_0:
						{
							levelid = 0;
							sb.destroy();
							return false;
						}
						case SDLK_1:
						{
							levelid = 1;
							sb.destroy();
							return false;
						}
						case SDLK_2:
						{
							levelid = 2;
							sb.destroy();
							return false;
						}
						case SDLK_3:
						{
							levelid = 3;
							sb.destroy();
							return false;
						}
						case SDLK_4:
						{
							levelid = 4;
							sb.destroy();
							return false;
						}
						case SDLK_5:
						{
							levelid = 5;
							sb.destroy();
							return false;
						}
						case SDLK_6:
						{
							levelid = 6;
							sb.destroy();
							return false;
						}
          case SDLK_7:
                  {
                          light_num = 0;
                          levelid = 7;
                          sb.destroy();
                          return false;
                  }
					}
					break;

				case SDL_MOUSEBUTTONDOWN:
					int x,y;
					switch(event.button.button)
					{
					case SDL_BUTTON_LEFT:
					//fprintf(stderr, "LEFT DOWN\n");
					x = event.button.x;
					y = event.button.y;
					gpoint.push_back(b2Vec2(x, y));
					break;
					case SDL_BUTTON_RIGHT:
					//fprintf(stderr, "RIGHT DOWN\n");
					if(gpoint.size() == 0) break;
					sb.addPolygon(gpoint, true);
					gpoint.clear();
					break;
					}
					break;
				default:
					break;
			}
		}

		display(dx,dy);
		int res = sb.simulateNextStep();
		if(res == 1)
		{
			sb.destroy();
			running = false;
			return true;
		}
		else if(res == -1)
		{
			sb.destroy();
			running = false;
			return false;
		}
		SDL_GL_SwapBuffers();
		if(1000.0/60>SDL_GetTicks()-start)
			SDL_Delay(1000.0/60-(SDL_GetTicks()-start));
	}
	return false;
}

void init()
{
	glMatrixMode(GL_PROJECTION);
	glOrtho(0,WIDTH,HEIGHT,0,-1,1);
	glMatrixMode(GL_MODELVIEW);
	glClearColor(0,0,0,1);
	init_func[0] = init_level0;
	init_func[1] = init_level1;
	init_func[2] = init_level2;
	init_func[3] = init_level3;
	init_func[4] = init_level4;
	init_func[5] = init_level5;
	init_func[6] = init_level6;
  init_func[7] = init_level7;
	color[0].r = 0.293, color[0].g = 0, color[0].b = 0.508;
	color[1].r = 0, color[1].g = 0.801, color[1].b = 0.398;
	color[2].r = 0, color[2].g = 0.801, color[2].b = 0;
}

int main(int argc,char** argv)
{
        now_time = time(0);
        SDL_Init(SDL_INIT_EVERYTHING);
        SDL_SetVideoMode(WIDTH,HEIGHT,32,SDL_OPENGL);

        init();
        srand(2);
        gameon = SDL_GetTicks();
        for(int i = 0; i < MAXLEVEL; ) {
                if(level(i))
                        ++i;
        }

        SDL_Quit();
        return 0;
}
