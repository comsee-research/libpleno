#include "viewer_3d.h"

#include <GL/gl.h>

void viewer_3d(v::ViewerContext& v, const PlenopticCamera& mfpc, tag::CameraBody)
{
    v.add_opengl([&mfpc, ref = mfpc.pose()](){
        glLineWidth(2);
        glEnable(GL_DEPTH_TEST);

        glBegin(GL_LINES);
            glColor3f(0.0, 0.0, 0.0);
            
            constexpr double border = 5.;
            const double dx = mfpc.sensor().width() * mfpc.sensor().scale() / 2. + border;
            const double dy = mfpc.sensor().height() * mfpc.sensor().scale() / 2. + border;
            const double dz = mfpc.sensor().pose().translation()[2];
            	
            const P3D p000{-dx,	-dy,	0.}; 	const P3D P000 = from_coordinate_system_of(ref, p000);
            const P3D p001{+dx,	-dy,	0.}; 	const P3D P001 = from_coordinate_system_of(ref, p001);
            const P3D p010{-dx,	+dy,	0.}; 	const P3D P010 = from_coordinate_system_of(ref, p010);
            const P3D p011{+dx,	+dy,	0.}; 	const P3D P011 = from_coordinate_system_of(ref, p011);
            const P3D p100{-dx,	-dy,	dz}; 	const P3D P100 = from_coordinate_system_of(ref, p100);
            const P3D p101{+dx,	-dy,	dz}; 	const P3D P101 = from_coordinate_system_of(ref, p101);
            const P3D p110{-dx,	+dy,	dz}; 	const P3D P110 = from_coordinate_system_of(ref, p110);
            const P3D p111{+dx,	+dy,	dz}; 	const P3D P111 = from_coordinate_system_of(ref, p111);
                        
            glAddLine(P000, P001);
            glAddLine(P000, P010);
            glAddLine(P000, P101);
            glAddLine(P000, P100);
            glAddLine(P000, P110);
            
            glAddLine(P001, P011);
            glAddLine(P001, P111);
            glAddLine(P001, P100);
            glAddLine(P001, P101);
            
            glAddLine(P011, P010);
            glAddLine(P011, P111);
            glAddLine(P011, P101);
            glAddLine(P011, P110);
            
            glAddLine(P100, P101);
            glAddLine(P100, P110);
            glAddLine(P100, P010);
            
            glAddLine(P101, P111);
            
            glAddLine(P110, P111);
            glAddLine(P110, P010);
        glEnd();
    });
}

void viewer_3d(v::ViewerContext& v, const CheckerBoard& gm, double scale)
{
    v.add_opengl([&gm, scale](){
        double transparency_value = 0.6;
        glColor4f(1.0,0.0,0.0, transparency_value);
        glPointSize(5.0);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST);

        glBegin(GL_POINTS);
            for(const P3D& n : gm)
                glAddPoint(n);
        glEnd();

        glBegin(GL_LINES);
        	draw_axis(gm.pose(), scale);
        	
            P3D tmp = gm.nodeInWorld(0);
            for(const P3D& n : gm)
            {
                glAddLine(n, tmp);
                tmp = n;
            }
        glEnd();
    });
}

void viewer_3d(v::ViewerContext& v, const PlenopticCamera& mfpc, tag::ThinLens, double scale)
{
	const ThinLensCamera& tcm = mfpc.main_lens();
	
    v.add_opengl([&tcm, scale, ref = from_coordinate_system_of(mfpc.pose(), tcm.pose())](){
        glLineWidth(2);
        glEnable(GL_DEPTH_TEST);

        glBegin(GL_LINES);
            draw_axis(ref, scale);
        glEnd();

    });
     
    v.add_opengl([&tcm, scale, ref = from_coordinate_system_of(mfpc.pose(), tcm.pose())](){
        double transparency_value = 0.6;
        glColor4f(0.0,1.0,0.0, transparency_value);
        glPointSize(1.0);
        glLineWidth(2.0);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST);
        
        float radius = tcm.diameter() / 2.f;
        float inc = 3.141592654f * 2.f / 50.f ;
        glBegin (GL_LINE_LOOP);
			for (float theta = 0.0f; theta < 3.141592654f * 2.f ; theta += inc)
			{
				P3D p{radius * std::cos(theta), radius * std::sin(theta), 0.};
            	glAddPoint(from_coordinate_system_of(ref, p));
			}
		glEnd();
		
		glBegin(GL_POINTS);
        	glColor4f(1.0,1.0,1.0, transparency_value);
			glPointSize(25.0);
            glAddPoint(from_coordinate_system_of(ref, P3D{0.,0., tcm.focal()}));
            glAddPoint(from_coordinate_system_of(ref, P3D{0.,0., -tcm.focal()}));
        glEnd();
        
        glBegin(GL_LINES);
        	glLineWidth(1.0);
        	glColor4f(0.0,0.0,0.0, transparency_value);
        	glAddLine(
        		from_coordinate_system_of(ref, P3D{0.,0., -60.}), 
        		from_coordinate_system_of(ref, P3D{0.,0., 60.})
        	);
        glEnd();
    });
}

void viewer_3d(v::ViewerContext& v, const PlenopticCamera& mfpc, tag::MLA, double scale)
{
	const MLA& gm = mfpc.mla();
	
	v.add_opengl([&gm, scale, ref = from_coordinate_system_of(mfpc.pose(), gm.pose())](){
        glLineWidth(2);
        glEnable(GL_DEPTH_TEST);

        glBegin(GL_LINES);
            draw_axis(ref, scale);
        glEnd();

    });
    
    v.add_opengl([&gm, scale, ref = mfpc.pose()](){
        double transparency_value = 0.6;
        glColor4f(0.0,0.0,1.0, transparency_value);
        glPointSize(1.0);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST);

        glBegin(GL_POINTS);
            for(const P3D& n : gm)
                glAddPoint(from_coordinate_system_of(ref,n));
        glEnd();

        glBegin(GL_LINES);        
            P3D tmp = gm.nodeInWorld(0);
            for(const P3D& n : gm)
            {
                glAddLine(from_coordinate_system_of(ref,n), from_coordinate_system_of(ref,tmp));
                tmp = n;
            }
        glEnd();
    });
}

void viewer_3d(v::ViewerContext& v, const PlenopticCamera& mfpc, tag::Sensor, double scale)
{
	const Sensor& s = mfpc.sensor();
	
    v.add_opengl([&s, scale, ref = from_coordinate_system_of(mfpc.pose(), s.pose())](){
        double transparency_value = 0.6;

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST); 
		
		
        glBegin(GL_LINES);
        	draw_axis(ref, scale);
        glEnd();
        
        glBegin(GL_QUADS);
            glColor4f(0.0, 0.0, 0.0, transparency_value); // black
            
            P3D tl{0.,0.,0.};
            P3D tr{s.width() * s.scale(),0.,0.};
            P3D bl{0.,s.height() * s.scale(),0.};
            P3D br{s.width() * s.scale(),s.height() * s.scale(),0.};
            
            glAddPoint(from_coordinate_system_of(ref, tl));
            glAddPoint(from_coordinate_system_of(ref, tr));
            glAddPoint(from_coordinate_system_of(ref, br));
            glAddPoint(from_coordinate_system_of(ref, bl));
            
        glEnd();
    }).update();
}


/**
 * @Brief draw a pose using OpenGL
 */
void viewer_3d(v::ViewerContext& v, const Pose& p, double scale)
{
    v.add_opengl([&p, scale](){
        glLineWidth(2);
        glEnable(GL_DEPTH_TEST);

        glBegin(GL_LINES);
            draw_axis(p, scale);
        glEnd();

    });
}

/**
 * @Brief draw some poses using OpenGL
 */
void viewer_3d(v::ViewerContext& v, const Poses& ps, double scale)
{
    v.add_opengl([&ps, scale](){
        glLineWidth(2);
        glEnable(GL_DEPTH_TEST);

        glBegin(GL_LINES);
            for (auto& p : ps)
                draw_axis(p, scale);
        glEnd();

    });
}

/**
 * @Brief viewer_3d draw a ray using OpenGL
 * the ray has its coordinates in world coordinates system
 */
void viewer_3d(v::ViewerContext& v, const Ray3D& r)
{
    v.add_opengl([&r](){
        glLineWidth(2);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST);

        glBegin(GL_LINES);
            glColor4f(r.color().r / 255.0,
                      r.color().g / 255.0,
                      r.color().b / 255.0,
                      r.color().a / 255.0
                     );
            glAddLine(r);
        glEnd();

    });
}

/**
 * @Brief viewer_3d draw some rays using OpenGL
 */
void viewer_3d(v::ViewerContext& v, const Rays3D& rs)
{
    v.add_opengl([&rs](){
        glLineWidth(2);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST);

        glBegin(GL_LINES);
            for (auto& r : rs)
            {
                glColor4f(r.color().r / 255.0,
                          r.color().g / 255.0,
                          r.color().b / 255.0,
                          r.color().a / 255.0
                         );
                glAddLine(r);
            }
        glEnd();

    });
}

/**
 * @Brief viewer_3d draw a point using OpenGL
 */
void viewer_3d(v::ViewerContext& v, const P3D& p, double scale)
{
    v.add_opengl([&p, scale](){
        double transparency_value = 0.6;
        glColor4f(1.0, 0.0, 0.0, transparency_value);
        glPointSize(scale);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST);

        glBegin(GL_POINTS);
            glAddPoint(p);
        glEnd();
    });
}

/**
 * @Brief viewer_3d draw some points using OpenGL
 */
void viewer_3d(v::ViewerContext& v, const P3DS& ps, double scale)
{
    v.add_opengl([&ps, scale](){
        double transparency_value = 0.6;
        glColor4f(1.0,0.0,0.0, transparency_value);
        glPointSize(scale);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST);

        glBegin(GL_POINTS);
            for (const P3D& p : ps)
                glAddPoint(p);
        glEnd();

    });
}

/**
 * @Brief viewer_3d draw some points using OpenGL
 */
void viewer_3d(v::ViewerContext& v, const PointsConstellation& points, double scale)
{
    v.add_opengl([&points, scale](){
        double transparency_value = 0.6;
        glColor4f(1.0,0.0,0.0, transparency_value);
        glPointSize(scale);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST);

        glBegin(GL_POINTS);
            for (const P3D& p : points.constellation)
                glAddPoint(p);
        glEnd();

    });
}

/**
 * @Brief viewer_3d draw a grid using OpenGL
 */
void viewer_3d(v::ViewerContext& v, const GridMesh3D& gm)
{
    v.add_opengl([&, gm](){
        double transparency_value = 0.6;
        glColor4f(1.0,0.0,0.0, transparency_value);
        glPointSize(5.0);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST);

        glBegin(GL_POINTS);
            for(const P3D& n : gm)
                glAddPoint(n);
        glEnd();

        glBegin(GL_LINES);
            P3D tmp = gm.nodeInWorld(0);
            for(const P3D& n : gm)
            {
                glAddLine(n, tmp);
                tmp = n;
            }
        glEnd();
    });
}

/**
 * @Brief viewer_3d draw a a plate in 3D using OpenGL
 */
void viewer_3d(v::ViewerContext& v, const Plate& plate)
{
    v.add_opengl([&plate](){
    	constexpr double scale = 0.25; // mm 
        constexpr double transparency_value = 0.6;
        glPointSize(5.0);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_DEPTH_TEST);
        
        glBegin(GL_LINES);
        	draw_axis(plate.pose(), 20.);
        glEnd();
		
        glBegin(GL_POINTS);
            for (double row = 0.; row < plate.height(); row+=scale) 
            {
                for (double col = 0.; col < plate.width(); col+=scale)
                {
                    const auto& color = plate.get_color(col, row);
                    const P3D& point = from_coordinate_system_of(plate.pose(), P3D{double(col), double(row), 0.0});
                    
                    glColor4f(color.r / 255., color.g / 255., color.b / 255., transparency_value);
                    glAddPoint(point);
                }
            }
        glEnd();
    });
}
