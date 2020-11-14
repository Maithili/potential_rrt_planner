#ifndef PRECOMPUTED_POTENTIAL_H
#define PRECOMPUTED_POTENTIAL_H

#include <Eigen/Dense>

#include "types.h"
#include "FlatPotential.h"


// Assuming center at 0,0
class PrecomputedPotential
{
public:

    PrecomputedPotential(OpenRAVE::EnvironmentBasePtr& env, float length_x, float length_y, float cell_size):
    cell_size_(cell_size), 
    n_x_(ceil(length_x/cell_size_)), 
    n_y_(ceil(length_y/cell_size_))
    {
        std::vector<OpenRAVE::KinBodyPtr> bodies;
        env->GetBodies(bodies);
        OpenRAVE::KinBodyPtr obstacles;
        for (OpenRAVE::KinBodyPtr& b: bodies)
        {
            if(b->GetName() == "obstacles")
                obstacles = b;
        }
        if(!obstacles.get())
            return;
        std::vector<OpenRAVE::KinBody::LinkPtr> links;
        links = obstacles->GetLinks();
        std::vector<OpenRAVE::KinBody::Link::GeometryPtr> obstacle_geometries_;
        for (OpenRAVE::KinBody::LinkPtr l : links)
        {
            std::vector<OpenRAVE::KinBody::Link::GeometryPtr> new_geoms = l->GetGeometries();
            obstacle_geometries_.insert(obstacle_geometries_.end(), new_geoms.begin(), new_geoms.end());
        }
        computeField(FlatPotential(obstacle_geometries_), env);
    }

    Location getPotentialGradientAt(Location x)
    {
        return gradient_field_[x_idx(x[0])][y_idx(x[1])];
    }

protected:
    void computeField(FlatPotential potential_calculator_, OpenRAVE::EnvironmentBasePtr& env);

    float x(int x_idx) { return (x_idx - (n_x_+1)/2 ) * cell_size_; }
    float y(int y_idx) { return (y_idx - (n_y_+1)/2 ) * cell_size_; }
    int x_idx(float x) { return static_cast<int>(x/cell_size_ + (n_x_+1)/2); }
    int y_idx(float y) { return static_cast<int>(y/cell_size_ + (n_y_+1)/2); }

    const float cell_size_;
    const int n_x_;
    const int n_y_;
    std::vector<std::vector<Location> > gradient_field_;
};

void PrecomputedPotential::computeField(FlatPotential potential_calculator_, OpenRAVE::EnvironmentBasePtr& env)
{
    // float max_grad = 0.0F;
    gradient_field_.clear();
    for (int ix=0; ix < n_x_ ; ++ix)
    {
        std::vector<Location> gradient_line;
		for (int iy=0; iy < n_y_ ; ++iy)
        {
            Location loc;
            loc << x(ix), y(iy);
			gradient_line.push_back(potential_calculator_.getPotentialGradientAt(loc));
            // max_grad = (max_grad<gradient_line.back().norm())?gradient_line.back().norm():max_grad;
		}
        gradient_field_.push_back(gradient_line);
        gradient_line.clear();
    }
    // for (int ix=0; ix < n_x_ ; ++ix)
    // {
	// 	for (int iy=0; iy < n_y_ ; ++iy)
    //     {
    //         float frac = (gradient_field_[ix][iy]).norm()/max_grad;
    //         float point[3]; point[0]=x(ix); point[1]=y(iy); point[2]=0.1;
    //         float color[3]; color[0]= frac; color[1]=0; color[2]=1-frac;
    //         viz_objects.push_back(env->plot3(point, 1, sizeof(point[0])*3, 5 ,color));
    //     }
    // }
    // char temp; std::cin>>temp;
    // viz_objects.clear();
}

#endif  // PRECOMPUTED_POTENTIAL_H