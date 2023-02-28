/**

\file
\author Datta Ramadasan
//==============================================================================
//         Copyright 2015 INSTITUT PASCAL UMR 6602 CNRS/Univ. Clermont II
//
//          Distributed under the Boost Software License, Version 1.0.
//                 See accompanying file LICENSE.txt or copy at
//                     http://www.boost.org/LICENSE_1_0.txt
//==============================================================================

*/

#ifndef __OPTIMISATION2_BA_BAS_HPP__
#define __OPTIMISATION2_BA_BAS_HPP__

#include "container.hpp"
#include <boost/mpl/transform.hpp>
#include <boost/fusion/include/as_map.hpp>
#include "create_hessian.hpp"

namespace lma
{
  
  template<class Bundle, class MatrixTag> struct ListJ
  {
    typedef typename Bundle::ListFunction L;
    
    template<class F> struct CreateBlocks
    {
      typedef Function<F> Fun;
      
      template<class Arg> struct CreateOneBlock
      {
        typedef  boost::fusion::pair<Arg*,typename lma::ContainerOption<
                                                          MatrixTag,
                                                          Size<typename Fun::ErreurType>::value,
                                                          lma::Size<Arg>::value
                                                         >::Matrix> type;
      };
      
      typedef typename br::as_map<typename mpl::transform<typename Fun::ParametersType,CreateOneBlock<mpl::_1>>::type>::type associative_tuple;
      typedef std::vector<associative_tuple,Eigen::aligned_allocator<associative_tuple>> type;
    };
    typedef typename mpl::transform<L,bf::pair<mpl::_1,CreateBlocks<mpl::_1>>>::type type;
  };
  
  template<class Bundle, class flt> struct Bas
  {
    typedef typename Bundle::ListFunction ListFunction;
    typedef typename Bundle::ParamFonctor ParamFonctor;
    typedef typename Bundle::ListeParam keys;
    
    typedef keys Keys;
    typedef flt MatrixTag;
    
    typedef typename mpl::transform< Keys, pair_>::type ListeDiag;
    typedef typename mpl::transform< Keys, VectorToPairStruct<mpl::_1,MatrixTag> >::type ListeVector;
    typedef typename ListH<Bundle,MatrixTag>::type ListeHessien;
    typedef typename br::as_map< ListeVector >::type Vectors;
    typedef typename br::as_map< ListeHessien >::type Hessian;

    typedef typename ListJ<Bundle,MatrixTag>::type ListeJacobian;
    typedef typename br::as_map< ListeJacobian>::type Jacobian;
    
    Hessian h;
    Vectors jte;
    Vectors delta;
    Jacobian jacob;
    
    Bas()
    {
//       std::cout << " hessien : " << ttt::name<ListeHessien>() << std::endl;
//       std::cout << ttt::name<ListeHessien2>() << std::endl;
//       std::cout << ttt::name<Jacobian>() << std::endl;
//       ListH<Bundle,MatrixTag>::disp();
    }
  };
}// end of bas

namespace ttt
{
}
#endif
