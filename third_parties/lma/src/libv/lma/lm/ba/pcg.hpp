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

#ifndef __OPTIMISATION2_BA_PCG_HPP__
#define __OPTIMISATION2_BA_PCG_HPP__

#include <libv/lma/time/tictoc.hpp>
#include <libv/lma/ttt/traits/wrap.hpp>
#include <libv/lma/ttt/traits/naming.hpp>
#include <libv/lma/lm/omp/omp.hpp>
#include <boost/fusion/include/as_map.hpp>
#include <boost/fusion/include/fold.hpp>
#include <boost/fusion/include/for_each.hpp>
#include <boost/mpl/for_each.hpp>
#include <boost/mpl/transform.hpp>

#include <iomanip>
#include <exception>

#include "make_type.hpp"
#include "utils.hpp"

namespace lma
{
  template<class T> bool is_zero_or_infinite(T x) { return ((x == 0.0) || (std::isinf(x))); }



  template<class S> struct AssignSameDiagInv
  {
    const S& s;
    AssignSameDiagInv(const S& s_):s(s_){}
    template<class Key, class Cont, template<class,class> class Pair> void operator()(Pair<Pair<Key,Key>,Cont>& obj) const
    {
      obj.second.set_diag_inv(boost::fusion::at_key<Pair<Key,Key>>(s));
    }

    template<class T> void operator()(T&) const {}
  };

  template<class D, class S> void assign_same_diag_inv(D& d, const S& s)
  {
    boost::fusion::for_each(d,AssignSameDiagInv<S>(s));
  }

  struct Minus
  {
    template<class Key, class Cont, template<class,class> class Pair> void operator()(Pair<Key,Cont>& obj) const
    { this->operator()(obj.second); }

    template<class T> void operator()(T& obj) const { obj.minus(); }
  };

  template<class A> struct Dot
  {
    typedef double result_type;
    const A& a;

    Dot(const A& a_):a(a_){}

    template<class Key,class Cont, template<class,class> class Pair> result_type operator()(const result_type& prev, const Pair<Key,Cont>& pair) const
    { return this->operator()<Cont>(prev,pair.second,boost::fusion::at_key<Key>(a)); }

    template<class Cont1, class Cont2> result_type operator()(const result_type& result, const Cont1& cont, const Cont2& b) const
    {
      return result + cont.compute_dot(b);
    }
  };

  template<class A> Dot<A> make_dot(const A& a) { return Dot<A>(a); }

  template<class P, class A, class B> struct ProdDiag21
  {
    P& p;
    const A& a;
    const B& b;
    ProdDiag21(P& p_, const A& a_, const B& b_):p(p_),a(a_),b(b_){}

    template<class Key> void operator()(ttt::wrap<Key>)
    {

      auto& residu = bf::at_key<Key>(p);
      const auto& refa = bf::at_key<bf::pair<Key,Key>>(a);
//       clement(a);
//       std::cout << ttt::name<Key>() << std::endl;
//       std::cout << ttt::name<A>() << std::endl;
      const auto& refb = bf::at_key<Key>(b);
      residu.resize(refb.size());
//       #pragma omp parallel for if(use_omp())
      for(auto i = residu.first() ; i < residu.size() ; ++i)
        residu(i) = refa(i,0) * refb(i);
//       prod_trig_sup(residu,refa,refb);
    }
  };

  template<class Scalar, class V> struct DecScalarProdV
  {
    const Scalar scalar;
    const V& v;

    DecScalarProdV(const Scalar& scalar_, const V& v_):scalar(scalar_),v(v_){}

    template<class Key, class Cont, template<class,class> class Pair> void operator()(Pair<Key,Cont>& pair) const
    {
      auto& result = pair.second;
      const auto& ref = boost::fusion::at_key<Key>(v);
      result.resize(ref.size());
//       #pragma omp parallel for if(use_omp())
      for(auto i = result.first() ; i < result.size() ; ++i)
        result(i) -= scalar * ref(i);
    }
  };
  
  template<class Scalar, class V> struct IncScalarProdV
  {
    const Scalar scalar;
    const V& v;

    IncScalarProdV(const Scalar& scalar_, const V& v_):scalar(scalar_),v(v_){}

    template<class Key, class Cont, template<class,class> class Pair> void operator()(Pair<Key,Cont>& pair) const
    {
      auto& result = pair.second;
      const auto& ref = boost::fusion::at_key<Key>(v);
      result.resize(ref.size());
//       #pragma omp parallel for if(use_omp())
      for(auto i = result.first() ; i < result.size() ; ++i)
        result(i) += scalar * ref(i);
    }
  };

  //     p = -y + beta * p;
  template<class A, class B, class C> struct MinusPlusScalarProdV
  {
    const A& a;
    const B b;
    const C& c;
    MinusPlusScalarProdV(const A& a_, const B& b_, const C& c_):a(a_),b(b_),c(c_){}

    template<class Key, class Cont, template<class,class> class Pair> void operator()(Pair<Key,Cont>& pair) const
    {
      auto& result = pair.second;
      const auto& ref1 = boost::fusion::at_key<Key>(a);
      const auto& ref2 = boost::fusion::at_key<Key>(c);

//       #pragma omp parallel for if(use_omp())
      for(auto i = result.first() ; i < result.size() ; ++i)
        result(i) =  - ref1(i) + b * ref2(i);
    }
  };

  template<class A, class B, class C> struct PlusScalarProdV
  {
    const A& a;
    const B b;
    const C& c;
    PlusScalarProdV(const A& a_, const B& b_, const C& c_):a(a_),b(b_),c(c_){}

    template<class Key, class Cont, template<class,class> class Pair> void operator()(Pair<Key,Cont>& pair) const
    {
      auto& result = pair.second;
      const auto& ref1 = boost::fusion::at_key<Key>(a);
      const auto& ref2 = boost::fusion::at_key<Key>(c);

//       #pragma omp parallel for if(use_omp())
      for(auto i = result.first() ; i < result.size() ; ++i)
        result(i) =  ref1(i) + b * ref2(i);
    }
  };
  
  template<class A, class B> struct AssignSame2
  {
    A& a;
    const B& b;
    AssignSame2(A& a_, const B& b_):a(a_),b(b_){}

    template<class Key> void operator()(ttt::wrap<Key>)
    {
      auto& refa = boost::fusion::at_key<Key>(a);
      const auto& refb = boost::fusion::at_key<Key>(b);

      if (refa.size() != refb.size())
      {
        std::cout << " DIFF ! " << ttt::name<Key>() << std::endl;
        std::cout << refa.name() << std::endl;
        std::cout << refb.name() << std::endl;
        std::cout << refa.size() << " , " << refb.size() << std::endl;
        getchar();
      }
      refa = refb;
      
      assert(!refa.is_invalid());
      
    }
  };

  template<class B, class Z> struct BMINUSZ
  {
    const B& b;
    const Z& z;
    BMINUSZ(const B& b_, const Z& z_):b(b_),z(z_){}

    template<class Key, class Cont, template<class,class> class Pair> void operator()(Pair<Key,Cont>& pair) const
    {
      auto& result = pair.second;
      const auto& ref1 = boost::fusion::at_key<Key>(b);
      const auto& ref2 = boost::fusion::at_key<Key>(z);

//       #pragma omp parallel for if(use_omp())
      for(auto i = result.first() ; i < result.size() ; ++i)
        result(i) =  ref1(i) - ref2(i);
    }
  };

  template<class Scalar, class V> IncScalarProdV<Scalar,V> incscalarprodv(const Scalar& a, const V& v) { return IncScalarProdV<Scalar,V>(a,v); }
  
  template<class Scalar, class V> DecScalarProdV<Scalar,V> decscalarprodv(const Scalar& a, const V& v) { return DecScalarProdV<Scalar,V>(a,v); }
  
  template<class A, class B, class C> MinusPlusScalarProdV<A,B,C> minus_plus_scalar_prod_v(const A& a, const B& b, const C& c){ return MinusPlusScalarProdV<A,B,C>(a,b,c); }
  
  template<class A, class B, class C> PlusScalarProdV<A,B,C> plus_scalar_prod_v(const A& a, const B& b, const C& c){ return PlusScalarProdV<A,B,C>(a,b,c); }
  
  template<class A, class B> AssignSame2<A,B> assign_same2(A& a, const B& b){ return AssignSame2<A,B>(a,b); }

  template<class B, class Z> BMINUSZ<B,Z> b_minus_z(const B& b, const Z& z) { return BMINUSZ<B,Z>(b,z); }
  struct SquaredNorm
  {
    typedef double result_type;
    template<class Key, class Cont, template<class,class> class Pair> result_type operator()(const result_type& prev, const Pair<Key,Cont>& obj) const
    {
//       std::cout << " Norm : " << prev << " " << obj.second.squaredNorm() << std::endl;
      return prev + obj.second.squaredNorm();
    }
  };

  template<class Tuple> double norm(const Tuple& t)
  {
    return std::sqrt(bf::fold(t,0.0,SquaredNorm()));
  }

  template<class Tuple> double dot(const Tuple& t)
  {
    return bf::fold(t,0.0,make_dot(t));
  }



  struct PcgConfig
  {
    double seuil;
    size_t max_iteration;
    size_t frequence;
    PcgConfig(double s = 0.9999, size_t m = 100, size_t f = -1):seuil(s),max_iteration(m),frequence(f){}
  };
  
  struct PCG : PcgConfig
  {
    PCG(PcgConfig config):PcgConfig(config) {}
    
    template<class Tag, class Container, class Delta>
    void operator()(const Container& cont, Delta& delta, const Tag&)
    {
      //Pour la notation et la méthode cf Méthode numérique appliquées, A. Gourdin, M. Boumahrat : page 258,259
      static const bool disp = false;

      typedef typename Tag::second_type Float;
      typedef typename Container::OptimizeKeys ListeParametre;
      typedef typename mpl::transform< 
				      ListeParametre, 
				      MakeTupleTable<mpl::_1,mpl::_1,Tag>
				     >::type ListeDiag;
      typedef typename mpl::transform<ListeParametre, VectorToPairStruct<mpl::_1,Tag> >::type ListeVector;
      typedef typename br::as_map<ListeVector>::type TupleResidu;
      typedef typename br::as_map<ListeDiag>::type TupleDiag;

      TupleDiag C;
      TupleResidu r,S,p,x;

      Float r0y0 = 0,r1y1 = 0;
      Float residualNorm2=std::numeric_limits<Float>::max(),residualNorm2Initial = 0;

      //r0 = b
      r = cont.B();// - mat * x;
  
      residualNorm2Initial = norm(r);//bf::fold(residual,0.0,dot(residual));
//       std::cout << " Initial norm : " << residualNorm2Initial << std::endl;
      
      if (residualNorm2Initial==0) return;

//       cont.get_preconditionner(C);
      
      assign_same_diag_inv(C,cont.A());// return le preconditionner inversé

      
      //p = C * r
//       bf::for_each(p,prod_diag_21(C,r));
      mpl::for_each<ListeParametre,ttt::wrap<mpl::_1>>(ProdDiag21<TupleResidu,TupleDiag,TupleResidu>(p,C,r));

      // S = p
      S = p;

      size_t max_element = nb_element(r);
      max_iteration = std::min(max_iteration,max_element);

      //if (cpt > max_element) std::cerr << __FILE__ << " : " << __LINE__ <<  " cpt > max_element : " << cpt << " > " << max_element << std::endl;

//       std::cout << " S \n " << to_vect(S).transpose() << std::endl;
      // r0y0 = r * S
      r0y0 = bf::fold(r,0.0,make_dot(S));
//       std::cout << " r0y0 " << r0y0 << std::endl;
      
      
//       std::cout << " Diff r0y0" << std::abs(to_vect(r).dot(to_vect(S)) - r0y0) <<  std::endl;
//       r0y0 = to_vect(r).dot(to_vect(S));
//       assert(  to_vect(r).dot(to_vect(S)) == r0y0 );
      if (r0y0==0) {std::cerr << " r0y0 = " << r0y0 << std::endl;return;}
        
      std::size_t it = 0;
// /*
      for( ; it < max_iteration ; ++it)
      {
        TupleResidu Ap;
        // AP = s * p
        // on fait le calcul ailleurs pour gérer les deux cas : S implicite ou explicite
        cont.prodAP(Ap,p);
// 	mpl::for_each<typename Container::TypesA,ttt::wrap<mpl::_1>>(prod_ap_p(Ap,cont.A(),p));

        // denom = p * Ap
        Float denom = bf::fold(p,0.0,make_dot(Ap));

//         std::cout << " Diff denom " << std::abs(to_vect(p).dot(to_vect(Ap)) - denom) <<  std::endl;
//         denom = to_vect(p).dot(to_vect(Ap));
        
        if (std::isinf(denom)) throw INF_ERROR("PCG denom");
  
        if (std::isnan(denom)) { std::cerr << color.bold() << color.red() <<  " PCG : p.dot(Ap) == NAN " << color.reset() << std::endl; throw NAN_ERROR("PCG: isnan(p*Ap)"); }
        if (denom==0) { std::cerr << color.bold() << color.red() <<  " PCG : p.dot(Ap) == 0 " << color.reset() << std::endl; throw NAN_ERROR("PCG: p*Ap==0"); }

//         std::cout << " numerateur   " << r0y0 << std::endl;
//         std::cout << " denominateur " << denom << std::endl;
    
        const Float alpha = r0y0 / denom;
  
        if (std::isinf(denom)) throw INF_ERROR("PCG alpha");
        


        if (it !=0 && it%frequence==0)
        {
//         si r dérive, on peut utiliser (mais c'est plus lent!) :
//         r = b - Ax
          TupleResidu Ax;
          cont.prodAP(Ax,x);
          bf::for_each(r,b_minus_z(cont.B(),Ax));
        }
        else
        {
        // r -= alpha * Ap
          bf::for_each(r,decscalarprodv(alpha,Ap));
        }

        residualNorm2 = norm(r);
//         std::cout << alpha << " ";
        // x += alpha * p
//         std::cout << to_vect(x) << std::endl << std::endl;
//         std::cout << to_vect(p) << std::endl;
//         auto x2 = (to_vect(x)  + alpha * to_vect(p)).eval();
        bf::for_each(x,incscalarprodv(alpha,p));
        
//         std::cout << (to_vect(x) - x2).transpose() << std::endl;
//         std::cout << " residu : " << residualNorm2 << std::endl;
        
        
        if (std::abs(residualNorm2-residualNorm2Initial)/residualNorm2Initial > seuil && residualNorm2<residualNorm2Initial)
        {
//           std::cout << " break condition " << std::endl;
          break;
        }
        
        // S = invM * r
// //         bf::for_each(S,prod_diag_21(C,r));
	  mpl::for_each<ListeParametre,ttt::wrap<mpl::_1>>(ProdDiag21<TupleResidu,TupleDiag,TupleResidu>(S,C,r));

        // r1y1 = r.dot(S);
        r1y1 = bf::fold(r,0.0,make_dot(S));
        
//         std::cout << " Diff r1y1 " << std::abs(to_vect(r).dot(to_vect(S)) - r1y1) <<  std::endl;
//         r1y1 = to_vect(r).dot(to_vect(S));
        //if (r1y1>r0y0) break;
        if(is_zero_or_infinite(r1y1)) {std::cout << " r1y1 is shitted ..." << std::endl;break;}//throw ZeroOrInfiniteError("PCG r1y1");
  
//         if(disp) std::cout << color.magenta() << " r1y1 = " << r1y1 << color.reset() << std::endl;
        
        const Float beta = r1y1 / r0y0;
        
        if(disp) std::cout << color.magenta() << " beta = " << beta << color.reset() << std::endl;

        if (is_zero_or_infinite(beta)) ZeroOrInfiniteError("PCG beta");
        // p = S + beta * p;
        bf::for_each(p,plus_scalar_prod_v(S,beta,p));

        r0y0 = r1y1;

//         std::cout << " critère : " << residualNorm2 << " / " << residualNorm2Initial << " = " << residualNorm2 / residualNorm2Initial << std::endl;
//         if (residualNorm2 / residualNorm2Initial < 0.1) break;
//         std::cout << " residual[" << it << "]: " << r1y1 << std::endl;
      }

//      std::cout << " nb iteration = " << it << " / " << max_iteration << std::endl;
//       std::cout <<  "final residual : " << residualNorm2 << std::endl;
//     std::cout << " FINAL [" << it << "]: " << r1y1 << std::endl;
      mpl::for_each<ListeParametre,ttt::wrap<mpl::_1>>(assign_same2(delta,x));
//       std::cout << " my pcg     :  " << to_matv(x).transpose() << std::endl;

    }
  };
}// eon

namespace ttt
{
  template<> struct Name<lma::PCG> { static std::string name(){ return "SSparsePCG"; } };
}

/*
void solve(const MatD& S, VecD& delta, const VecD& E, const std::size_t& nb_camera)
{
  const std::size_t size = nb_camera * C;

  double seuil = 1e-16;   //à choisir entre 1e-8 et 1e-16 (influe sur le nombre d'itérations du PCG et sa précision)

  double residual, residual_first;

  VecD r = -E;
  VecD x = VecD::Zero(size);
  residual_first = r.dot(r);

  VectorCC invMv(nb_camera);
  for(std::size_t i = 0 ; i < nb_camera ; ++i)
    invMv[i] = S.inverse();

  VecD y = VecD::Zero(size);

  for(std::size_t i = 0 ; i < nb_camera ; ++i)
    BlockC1::view(y,i*C,0) = invMv[i] * BlockC1::view(r,i*C,0);

  VecD p = - y;
  double r0y0 = r.dot(y), r1y1;

  for(std::size_t it = 0 ; it < size ; ++it)
  {
    VecD Ap = S * p;

    double temp = p.dot(Ap) ;

    if (std::isnan(temp)) { V_TXT(" PCG : p.dot(Ap) == NAN "); break; }

    const double alpha = r0y0 / temp;

    x += alpha * p;
    r += alpha * Ap;
    residual = r.dot(r);

    if ( residual < seuil * residual_first ) {break;}

    for(std::size_t i = 0 ; i < nb_camera ; ++i)
      BlockC1::view(y,i*C,0) = invMv[i] * BlockC1::view(r,i*C,0);

    r1y1 = r.dot(y);

    const double beta = r1y1 / r0y0;
    p = -y + beta * p;

    r0y0 = r1y1;
  }
  delta = x;
}
*/


#endif
