#pragma once

#include <Eigen/Core>

#include <libv/core/miscmath.hpp>

#include "graphic/gui.h"
#include "graphic/viewer_2d.h"

#include "io/printer.h"
#include "io/colors.h"

#include "processing/improcess.h"

template<typename ImageType>
struct GenericLevenbergMarquardt
{
    double max_it_;
    double min_err_;
    ImageType observation;
    ImageType initial;
    ImageType mask;

    GenericLevenbergMarquardt(const double mx_it = 1e3,
                              const double m_e = 0.0,
                              const ImageType& o = ImageType(),
                              const ImageType& i = ImageType(),
                              const ImageType& m = ImageType())
    : max_it_(mx_it), min_err_(m_e)
    {
        assert(o.size() == i.size() && o.size() == m.size());

        observation = o.clone();
        initial = i.clone();
        mask = m.clone();
    }

    ImageType compute_prediction(const Transformation& t) const;
    Eigen::MatrixXd compute_jacobian(const Transformation& t) const;
    Eigen::MatrixXd compute_hessian(const Eigen::MatrixXd& jacobian) const;

    void add_lambda_to_hessian(Eigen::MatrixXd& hessian, const double lambda) const;

    Eigen::VectorXd compute_error(const Transformation& h) const;
    Eigen::VectorXd solve_equation(const Eigen::MatrixXd& jacobian, const Eigen::VectorXd& error,
                                   const double lambda) const;

    std::tuple<int, double> run(Transformation& h, double lambda, bool verbose = false);
};

template<typename ImageType>
ImageType GenericLevenbergMarquardt<ImageType>::compute_prediction(const Transformation& t) const
{
    ImageType warped;
    warp(t, initial, warped);

    return warped;
}

//function computing the jacobian of the error
template<typename ImageType>
Eigen::MatrixXd GenericLevenbergMarquardt<ImageType>::compute_jacobian(const Transformation& t) const
{
    const size_t param_to_optimize = t.mode;

    // the jacobian has n_parameters (DOF) column and h * w number of lines
    Eigen::MatrixXd jacobian(size(observation), param_to_optimize);
    jacobian.setZero();

    // ImageType derivatives
    ImageType warped = compute_prediction(t);

    //Computing gradients the image
    ImageType x_grad, y_grad;
    x_gradient<double>(warped, x_grad);
    y_gradient<double>(warped, y_grad);

    // Derivative of warping
    Eigen::Matrix<double, 3, 9> d_warping = Eigen::Matrix<double, 3, 9>::Zero(); // aka J_w

    // Derivative of transformation
    Eigen::MatrixXd d_transformation = get_transformation_jacobian(t);

    // building the jacobian
    Eigen::MatrixXd jac_intermediaire (3, param_to_optimize);
    jac_intermediaire.setZero();

    int index = 0;
    for (size_t row = 0; row < height(observation); ++row)
        for (size_t col = 0; col < width(observation); ++col)
        {
            d_warping <<
             col,  row, 1.0,  0.0,  0.0,  0.0, -col * col, -col * row, -col * 1.0,
             0.0,  0.0, 0.0,  col,  row,  1.0, -row * col, -row * row, -row * 1.0,
             0.0,  0.0, 0.0,  0.0,  0.0,  0.0,        0.0,        0.0,        0.0;

            jac_intermediaire = d_warping * d_transformation;

            //J_tot = J_I * J_w * J_G
            for (size_t p = 0; p < param_to_optimize; ++p)
            {
                jacobian(index, p) = get_pixel<double>(x_grad, col, row) * jac_intermediaire(0, p)
                                   + get_pixel<double>(y_grad, col, row) * jac_intermediaire(1, p);
            }

            ++index;
        }

    return jacobian;
}

template<typename ImageType>
Eigen::MatrixXd GenericLevenbergMarquardt<ImageType>::compute_hessian(const Eigen::MatrixXd& jacobian) const
{
    Eigen::MatrixXd hessian = jacobian.transpose() * jacobian;

    return hessian;
}

template<typename ImageType>
void GenericLevenbergMarquardt<ImageType>::add_lambda_to_hessian(Eigen::MatrixXd& hessian,
                                                             const double lambda) const
{
    for (int i = 0; i < hessian.cols(); ++i)
        hessian(i, i) += lambda;
}

template<typename ImageType>
Eigen::VectorXd GenericLevenbergMarquardt<ImageType>::compute_error(const Transformation& t) const
{
    const ImageType prediction = compute_prediction(t);

    Eigen::VectorXd errors (size(observation));
    errors.setZero();

    int pixel_index = 0;
    for (size_t row = 0; row < height(observation); ++row)
    {
        for (size_t col = 0; col < width(observation); ++col)
        {
            if (	get_pixel<double>(mask, col, row) != 0.0 // disacarding pixels out of mask
            	and get_pixel<double>(prediction, col, row) != -1.0
            )
            {
                errors(pixel_index) = get_pixel<double>(observation, col, row) 
                                           	- get_pixel<double>( prediction, col, row);
            }
            else
            {
                // pas de mallus :*
            }

            ++pixel_index;
        }
    }

    return errors;
}

template<typename ImageType>
Eigen::VectorXd GenericLevenbergMarquardt<ImageType>::solve_equation(const Eigen::MatrixXd& jacobian,
                                                                 const Eigen::VectorXd& error,
                                                                 const double lambda) const
{
    Eigen::MatrixXd hessian = compute_hessian(jacobian);
    add_lambda_to_hessian(hessian, lambda);

#if 0
    if ( hessian.determinant() < 1e-3 and hessian.determinant() > -1e-3)
    {
         std::cout << std::string(100, '-') << std::endl;
         std::cout << yellow("GenericLevenbergMarquardt::solve_equation: ");
         std::cout << yellow("Mauvais conditionnement !") << std::endl;
         std::cout << "jacobian:\n" << jacobian << std::endl;
         std::cout << "\n\n\n" << std::endl;
         std::cout << "hessian:\n" << hessian << std::endl;
         std::cout << "\n\n\n" << std::endl;
         std::cout << "determinant: " << hessian.determinant() << std::endl;
         std::cout << "\n\n\n" << std::endl;
         std::cout << "lambda: " << lambda << std::endl;
         std::cout << std::string(100, '-') << std::endl;
    //     std::getchar();
    }
#endif

    return hessian.fullPivLu().solve( jacobian.transpose() * error);
}

template<typename ImageType>
std::tuple<int, double> GenericLevenbergMarquardt<ImageType>::run(Transformation& t, double lambda, bool verbose)
{
    double v = 2.0;
    bool accepted = false;

    int iteration_number = 0;

    if(verbose)
    {	
    	PRINT_DEBUG("=== Running GenericLevenbergMarquardt (GLM) Optimization");
		PRINT_DEBUG(	
					"INITIAL PARAMETERS:" << std::endl
				<< 	"\tTransformation:\n" << t()
		);
    
    	PRINT_DEBUG("#\tlambda\t\terror\t\tdE\t\tdelta.norm()");
    	PRINT_DEBUG(std::string(80, '-'));
	}

    double previous_cost = 0.0;
    double new_cost = 0.0;

    Eigen::VectorXd errors (size(observation));
    errors.setZero();
    Eigen::VectorXd delta (6);
    delta.setZero();

    Eigen::Matrix<double, 3, 3> tmp_matrix = Eigen::Matrix<double, 3, 3>::Identity();

    // compute the initial cost
    errors = compute_error(t);
    new_cost = errors.squaredNorm();

    // displaying initial information
    if(verbose)
    {
		PRINT_DEBUG(
			 	0 << "\t"
			<< std::scientific << std::setprecision(1) << lambda << "\t"
			<< std::setprecision(3) << new_cost << "\t\t"
			<< std::setprecision(3) << 0 << "\t\t\t"
			<< std::setprecision(1) << delta.norm() << "\t"
		);
	}
    // storing it in the new cost
    previous_cost = new_cost;
    if (std::isnan(new_cost))
    {
        PRINT_ERR("GLM: initial cost is NaN.");
        return {0, INFINITY};
    }

    for (size_t i = 0; i < max_it_; ++i)
    {
        // computing the new state
        delta = solve_equation(compute_jacobian(t), errors, lambda);

        // on passe delta dans une exponentiel map
        tmp_matrix = t(); // tmp_matrix stock l'état courant de h
        t.total_matrix = t.total_matrix * get_exponential_map(t, delta);

        // computing the new error
        errors = compute_error(t);
        new_cost = errors.squaredNorm();

        if (std::isinf(new_cost) or new_cost > 1e+100)
        {
            PRINT_ERR("GLM: new_cost is INF.");
            return {i, INFINITY};
        }

        if (std::isnan(new_cost))
        {
            PRINT_ERR("GLM: new_cost is NaN.");
            return {i, INFINITY};
        }

        // comparing errors
        if (new_cost < previous_cost)
        {
            accepted = true;
            // previous_cost = new_cost;

            lambda /= v;
            // lambda = v::clamp(lambda, 1e-5, 1e5);
        }
        else
        {
            accepted = false;

            lambda *= v;
            // lambda = v::clamp(lambda, 1e-5, 1e5);

            t.total_matrix = tmp_matrix;
        }

		if(verbose)
		{
		    if (accepted)
		    {
		        PRINT_DEBUG(
		        		green(i) << "\t"
		        	<< std::scientific << std::setprecision(1) << green(lambda) << "\t"
		       		<< std::setprecision(3) << green(new_cost) << "\t\t"
		        	<< std::setprecision(3) << green(previous_cost - new_cost) << "\t\t\t"
		        	<< std::setprecision(1) << green(delta.norm()) << "\t"
		        );
		    }
		    else
		    {
		        PRINT_DEBUG(
		        		red(i) << "\t"
		        	<< std::scientific << std::setprecision(1) << red(lambda) << "\t"
		        	<< std::setprecision(3) << red(new_cost) << "\t\t"
		        	<< std::setprecision(3) << red(new_cost - previous_cost) << "\t\t\t"
		        	<< std::setprecision(1) << red(delta.norm()) << "\t"
		        );
		    }
		
			GUI(
		        double vw = width(observation);
		        double vh = height(observation);
				
				Viewer::stash();
				
		        Image display_observation;
		        observation.convertTo(display_observation, CV_8UC1);
		        // std::cout << display_observation << std::endl;
		        RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()).name("observation"), display_observation, 0, 0);
		        Viewer::update();
		        
		        Image display_mask;
		        mask.convertTo(display_mask, CV_8UC1);
		        // std::cout << display_mask << std::endl;
		        RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()).name("mask"), display_mask, 0, (vh + 10));            
		        Viewer::update();
		        
		        Image display_initial;
		        initial.convertTo(display_initial, CV_8UC1);
		        RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()).name("initial"), display_initial, vw + 10, 0);
		        Viewer::update();

		        ImageType warped = compute_prediction(t);
		        Image display_warped;
		        warped.convertTo(display_warped, CV_8UC1);
		        RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()).name("warped"), display_warped, 2 * (vw + 10), 0);
		        Viewer::update();


		        Image display_filtered_warped = display_warped - display_mask;
		        RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()).name("filtered_warped"), display_filtered_warped, 3 * (vw + 10), 0);
		        Viewer::update();
	#if 0
		        ImageType difference = warped - observation;
		        Image display_difference;
		        difference.convertTo(display_difference, CV_8UC1);
		        RENDER_DEBUG_2D(Viewer::context().layer(Viewer::layer()).name("difference"), display_difference, 2 * (vw + 10), (vh + 10));
		        Viewer::update();
	#endif            
		        Viewer::pop();
		        
			);
		
		}
		
        bool should_stop = false;

        // stop conditions
        // TODO: faire une fonction LevenbergMarquardt::stop() const avec un
        // seul if()
        if (i >= max_it_)
        {
            if(verbose) PRINT_DEBUG("i >= max_it_");

            should_stop = true;
        }

        if (new_cost <= min_err_)
        {
            if(verbose)PRINT_DEBUG("new_cost <= min_err_");

            should_stop = true;
        }
        if (previous_cost == new_cost)
        {
            if(verbose) PRINT_DEBUG("previous_cost == new_cost");
            should_stop = true;
        }
        if (delta.norm() < 1e-7)
        {
            if(verbose) PRINT_DEBUG("delta.norm() < 1e-7");

            should_stop = true;
        }

        if (accepted)
            previous_cost = new_cost;

        if (should_stop)
        {
            if(verbose) PRINT_DEBUG("Converging in " << i + 1 << " iterations !");

            iteration_number = i;

            break;
        }
    }
	
	if(verbose)
    {
		PRINT_DEBUG(
				"FINAL PARAMETERS:" << std::endl
			<< "\tTransformation:\n" << t()
		);
	}
    return {iteration_number, previous_cost};
}
