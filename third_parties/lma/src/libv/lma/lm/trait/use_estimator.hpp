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

#ifndef __LMA_OPT2_TRAIT_USE_ESTIMATOR_HPP__
#define __LMA_OPT2_TRAIT_USE_ESTIMATOR_HPP__

#include <libv/lma/lm/container/container.hpp>
#include <boost/type_traits/is_convertible.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/mpl/bool.hpp>
#include <libv/lma/numeric/mediane.hpp>
#include <iostream>

namespace lma
{
  struct MEstimator_{};
  
  template<class Base> struct MEstimator : MEstimator_
  {
    Base const& cast() const {return static_cast<Base const &>(*this); }
    
    template<class Float, size_t N>
    Eigen::Matrix<Float,N,1> me(const std::array<Float,N>& res, Float C) const
    {
      Eigen::Matrix<Float,N,1> ret;
      for(size_t i = 0 ; i < N ; ++i)
        ret[i] = cast().weight(res[i],C);
      return ret;
    }
    
    template<class Float>
    Eigen::Matrix<Float,1,1> me(const Float& res,const Float& C) const
    {
      Eigen::Matrix<Float,1,1> ret;
      ret << cast().weight(res,C);
      return ret;
    }
    
    template<class Float, int N>
    Eigen::Matrix<Float,N,1> me(const Eigen::Matrix<Float,N,1>& res, Float C) const
    {
      Eigen::Matrix<Float,N,1> ret;
      for(int i = 0 ; i < N ; ++i)
        ret[i] = cast().weight(res[i],C);
      return ret;
    }
  };
  
  namespace detail
  {
    template<class F> struct IsMEstimator : boost::is_convertible<F*,MEstimator_*>::type {};
  
    template<class W> struct Weight_J
    {
      const W& weight;
      Weight_J(const W& w_):weight(w_){}
      
      template<class Pair> void operator()(Pair& pair) const
      {
        for(int i = 0 ; i < Rows<decltype(pair.second)>::value ; ++i)
          for(int j = 0 ; j < Cols<decltype(pair.second)>::value ; ++j)
            pair.second(i,j) = pair.second(i,j)*weight[i];
      }
    };

    template< class F, class Jacobs, class Erreurs, class Mad>
    void apply_mestimator(const F& f, Jacobs& jacobs, Erreurs& erreur, const Mad& mad, typename boost::enable_if<detail::IsMEstimator<F>>::type* =0)
    {
      auto weight  = f.me(erreur,boost::fusion::at_key<F>(mad));
//       std::cout << " erreur " << erreur[0] << ", " << weight <<  std::endl;
      cwise_product(erreur,weight,erreur);
      boost::fusion::for_each(jacobs,Weight_J<decltype(weight)>(weight));
    }
    
    template<class F, class Jacobs, class Erreurs, class Mad> void apply_mestimator(const F&, Jacobs&, Erreurs&, const Mad&,typename boost::disable_if<detail::IsMEstimator<F>>::type* =0) { }
    
    template<class F, class Erreurs, class Mad>
    void apply_mestimator_erreur(const F& f, Erreurs& erreur, const Mad& mad, typename boost::enable_if<detail::IsMEstimator<F>>::type* =0)
    {
      auto weight  = f.me(erreur,boost::fusion::at_key<F>(mad));
      cwise_product(erreur,weight,erreur);
    }
    
    template<class F, class Erreurs, class Mad> void apply_mestimator_erreur(const F&, Erreurs&, const Mad&, typename boost::disable_if<detail::IsMEstimator<F>>::type* =0) { }
    
  }
  
  
  template<class Float>
  struct GermanMcClure : MEstimator<GermanMcClure<Float>>
  {
    Float coeff_mad;
    GermanMcClure(Float a_):coeff_mad(a_){}

    Float weight(const Float& res, const Float& C) const
    {
      return (C != 0 ? (C / (res*res+C*C)) : 1.0);
    }
    
    Float compute(std::vector<Float> norms) const
    {
      if (norms.empty())
        return 0;

      Float med = mediane(norms);

      for(Float& m : norms)
        m = std::abs(m-med);
      
      Float MAD = mediane(norms);
      
      Float C = med + coeff_mad * MAD;
      // std::cout << " C : " << C << " = " << med << " + " << coeff_mad << " * " << MAD << std::endl;
      return C;
    }
  };
}

#endif
