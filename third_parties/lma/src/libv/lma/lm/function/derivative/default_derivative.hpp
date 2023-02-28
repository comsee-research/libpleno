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

#ifndef __OPTIMISATION2_FUNCTION_DERIVATIVE_DEFAULT_DERIVATIVE_HPP__
#define __OPTIMISATION2_FUNCTION_DERIVATIVE_DEFAULT_DERIVATIVE_HPP__

#include "default_derivative_tag.hpp"
#include "analytical_derivative.hpp"
#include "numerical_derivative.hpp"
#include "automatic_derivative.hpp"
#include "detail_derivative.hpp"

namespace lma
{
  template<class Obs> struct IfDerivativeIsNotChoosen :
    boost::mpl::not_<
      boost::mpl::or_<
                      boost::is_convertible<Obs*,NumericForward*>,
                      boost::is_convertible<Obs*,NumericCentral*>,
                      boost::is_convertible<Obs*,Analytical*>,
                      boost::is_convertible<Obs*,Automatic*>,
                      boost::is_convertible<Obs*,CheckAnalytical*>
                     >
                    > {};

  template<class F, class D> struct EnableIfConvertible : boost::enable_if<boost::is_convertible<F*,D*>> {};

  template<class Tag, class Fonctor, class Tuple, class Jacob, class Erreur>
  void derivator(const lma::Function<Fonctor>& fonctor, const Tuple& tuple, Jacob& result, const Erreur& error, typename EnableIfConvertible<Fonctor,NumericForward>::type* =0) // numeric forward
  {
    NumericalDerivator<Tag>::derive(fonctor,tuple,result,error);
  }
  
  template<class Tag, class Fonctor, class Tuple, class Jacob, class Erreur>
  void derivator(const lma::Function<Fonctor>& fonctor, const Tuple& tuple, Jacob& result, const Erreur&, 
    typename boost::enable_if<
                                boost::mpl::or_<
                                                  boost::is_convertible<Fonctor*,NumericCentral*>,
                                                  IfDerivativeIsNotChoosen<Fonctor>
                                               >
                             >::type* =0) // numeric central
  {
    NumericalDerivator<Tag>::derive(fonctor,tuple,result);
  }

  template<class Tag, class Fonctor, class Tuple, class Jacob, class Erreur>
  void derivator(const lma::Function<Fonctor>& fonctor, const Tuple& tuple, Jacob& result, const Erreur&, typename EnableIfConvertible<Fonctor,Analytical>::type* =0) // analytic
  {
    AnalyticalDerivator<Tag>::derive(fonctor,tuple,result);
  }

  template<class Tag, class Fonctor, class Tuple, class Jacob, class Erreur>
  void derivator(const lma::Function<Fonctor>& fonctor, const Tuple& tuple, Jacob& result, const Erreur& error, typename EnableIfConvertible<Fonctor,CheckAnalytical>::type* =0) // check_analytic
  {
    NumericalDerivator<Tag>::derive(fonctor,tuple,result,error);
    AnalyticalDerivator<Tag>::derive(fonctor,tuple,result);
  }

  template<class Tag, class Fonctor, class Tuple, class Jacob, class Erreur>
  void derivator(const lma::Function<Fonctor>& fonctor, const Tuple& tuple, Jacob& result, const Erreur&, typename EnableIfConvertible<Fonctor,Automatic>::type* =0) // automatic
  {
    AutomaticDerivator<Tag>::derive(fonctor,tuple,result);
  }
}

namespace ttt
{
  //template<class Tag> struct Name<lma::Derivator<Tag>> { static std::string name(){ return std::string()+"Derivator<" + ttt::name<Tag>() + ">"; } };
}

#endif
