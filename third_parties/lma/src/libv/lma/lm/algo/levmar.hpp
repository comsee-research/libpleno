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

#ifndef __OPTIMISATION2_ALGO_LM_MANY_CLASSES_LEVMAR_HPP__
#define __OPTIMISATION2_ALGO_LM_MANY_CLASSES_LEVMAR_HPP__

#include <boost/fusion/include/for_each.hpp>
#include <boost/mpl/copy_if.hpp>
#include <libv/lma/lm/ba/bas.hpp>
#include <libv/lma/lm/ba/initialize.hpp>
#include <libv/lma/lm/ba/fill_hessien.hpp>
#include <libv/lma/lm/bundle/cost_and_save.hpp>
#include <libv/lma/lm/container/map_container.hpp>
#include <libv/lma/lm/container/container.hpp>
#include <libv/lma/lm/function/function.hpp>
#include <libv/lma/lm/trait/use_estimator.hpp>
#include <libv/lma/time/tictoc.hpp>

namespace lma
{
  template<class F> struct MakeResidualVector : bf::pair<F,AlignVector<std::pair<typename Function<F>::ErreurType,bool>>> {};

  template<> struct MakeResidualVector<mpl::_1>{};

  template<class A, class A_, class B, class B_, class C, class C_>
  void prod_v_v_h(VectorColumn<A,A_>& a, const VectorColumn<B,B_>& b, const DiagonalTable<C,C_>& c)
  {
    for(auto i = c.first() ; i < c.size() ; ++i)
      a(i) += b(i) * c(i);
  }
  
  template<class A, class A_, class B, class B_, class _>
  void prod_v_v_h(VectorColumn<B,B_>& r, const VectorColumn<A,A_>& delta, const Table<A,B,_>& h)
  {
    for(auto i = h.first() ; i < h.size() ; ++i)
      for(auto j = h.first(i) ; j < h.size(i) ; ++j)
        r(h.indice(i,j)) += delta(i) * h(i,j);
  }
  
  template<class A, class A_, class B, class B_, class _>
  void prod_v_v_h_t(VectorColumn<A,A_>& r, const VectorColumn<B,B_>& delta, const Table<A,B,_>& h)
  {
    for(auto i = h.first() ; i < h.size() ; ++i)
      for(auto j = h.first(i) ; j < h.size(i) ; ++j)
        r(i) += delta(h.indice(i,j)) * transpose(h(i,j));
  }
  
  template<class A, class B, class C, class C_>
  void prod_v_v_h_dispatch(A& r, const B& delta, const DiagonalTable<C,C_>& c)
  {
    prod_v_v_h(bf::at_key<C>(r),bf::at_key<C>(delta),c);
  }
  
  template<class A, class B, class C1, class C2, class _>
  void prod_v_v_h_dispatch(A& r, const B& delta, const Table<C1,C2,_>& c)
  {
    prod_v_v_h(bf::at_key<C2>(r),bf::at_key<C1>(delta),c);
    prod_v_v_h_t(bf::at_key<C1>(r),bf::at_key<C2>(delta),c);
  }

  struct GetMax
  {
    double& lambda;
    GetMax(double& d):lambda(d){}
    template<class Table> void operator()(const Table& table){ lambda = std::max(lambda,table.max_lambda());}
  };

  struct ComputeScale
  {
    double &scale;
    double &lambda;
    ComputeScale(double& scale_, double& lambda_):scale(scale_),lambda(lambda_){}
    template<class Delta, class Jte> void operator() (const Delta& delta, const Jte& jte)
    {
      for(auto i = delta.first() ; i < delta.size() ; ++i)
        for(size_t k = 0 ; k < delta.I; ++k)
          scale += (delta(i)[k] * (lambda * delta(i)[k] + jte(i)[k]));
    };
  };
    
  template<class Policy> struct LevMar : Policy
  {
    typedef LevMar<Policy> type;
    
    typedef utils::Tic<false> Tic;
    typedef typename Policy::Bundle Bundle;
    typedef typename Policy::Ba Ba;
    typedef typename Ba::Keys Keys;

    Ba ba_;
    const Ba& ba() const { return ba_; }

    double residual_evaluations;
    double jacobian_evaluations;
    double norm_eq_;
    double preprocess;

    double prev_lambda;
    double erreur_,previous_erreur_;
    double nb_used_fonctor;

    typedef MultiContainer<typename Bundle::ListFunction,MakeResidualVector<mpl::_1>> ContainerMapErreur;

    utils::Tic<true> free_tic;

    ContainerMapErreur map_erreur;
    
    static std::string name(){ return ttt::name<LevMar<Policy>>(); }
    
    template<class Config>
    LevMar(Config config):Policy(config),prev_lambda(-1.0),erreur_(-1.0),previous_erreur_(-1.0)
    {
      residual_evaluations = 0;
      preprocess = norm_eq_ = jacobian_evaluations = 0;
    }

    typedef typename
    mpl::copy_if<
                  typename Bundle::ListFunction,
                  detail::IsMEstimator<mpl::_1>
                >::type MEstimatorList;
                
    typedef typename 
      br::as_map<
                  typename mpl::transform<
                                          MEstimatorList,
                                          bf::pair<mpl::_1,double>
                                         >::type
                >::type Meds;
    Meds meds;//bf::tuple< pair<F1,double>, pair<F2,double> ...>

    void init(Bundle& bundle_)
    {
      free_tic.tic();
      bundle_.update();
      initialize(bundle_,ba_);
      Policy::init(bundle_,ba_);
      cost_and_save_mad<Meds>(bundle_,meds);
      preprocess = free_tic.toc();
    }

    void restore_erreur()
    {
      assert(erreur_!=-1.0);//! l'erreur a déjà été calculé au moins une fois
      erreur_ = previous_erreur_;
    }

    double get_erreur() const
    {
      assert(erreur_!=-1.0);
      return erreur_;
    }

    
    std::pair<double,int> compute_erreur(const Bundle& bundle_)
    {
      free_tic.tic();
      if (erreur_!=-1.0)
        previous_erreur_ = erreur_;
      std::tie(erreur_,nb_used_fonctor) =  cost_and_save(bundle_,map_erreur,meds);
      if (erreur_==-1)
	     std::cerr << " LMA::compute_erreur " << erreur_ << " " << previous_erreur_ << std::endl;
      assert(erreur_!=-1.0);
      residual_evaluations += free_tic.toc();
      return {erreur_,nb_used_fonctor};
    }

    double compute_scale(double lambda) const
    {
      double scale = 0.0;
      ComputeScale cs(scale,lambda);
      for_each<MetaBinary<typename Ba::Keys>>(std::tie(ba_.delta,ba_.jte),cs);
      return scale;
    }
    
    double init_lambda()
    {
      double lambda = 0;
      GetMax gm(lambda);
//       for_each<MetaUnary<typename Ba::ListeDiag>>(std::tie(ba_.h),[&lambda](auto& table){ lambda = std::max(lambda,table.max_lambda());});
      for_each<MetaUnary<typename Ba::ListeDiag>>(std::tie(ba_.h),gm);
      return lambda * 1e-5;
    }
    
//     double prediction(const Bundle& bundle_)
//     {
//       double pred=0;
//       double alpha=0;
//       typename Ba::Vectors v = ba_.jte;
//       
//       bf::for_each(v,[](auto& pair){ pair.second.set_zero(); });
//       bf::for_each(ba_.h,[&](auto& pair){prod_v_v_h_dispatch(v,ba_.delta,pair.second);});
//       for_each<MetaBinary<typename Ba::Keys>>(std::tie(v,ba_.delta),[&alpha](auto& a, auto& b){
//         for(auto i = a.first() ; i < a.size() ; ++i)
//           alpha += a(i)*b(i);
//       });
//       alpha /= 2.0;
//       std::cout << " alpha : " << alpha << std::endl;
// //       abort();
//       bf::for_each(ba_.jacob,
//         [&](auto& pair)
//         {
//           typedef typename std::decay<decltype(pair)>::type Pair;
//           typedef typename Pair::first_type Obs;
//           auto& jacobs = pair.second;
//           TooN::Vector<2,double> vec = TooN::Ones;
//           vec *= alpha;
//                 
//           for(size_t iobs = 0 ; iobs < jacobs.size() ; ++iobs)
//           {
//             bf::for_each(jacobs.at(iobs),
//               [&]
//               (auto& pair_block_jacob)
//               {
//                 typedef typename std::decay<decltype(pair_block_jacob)>::type PairBlockJacob;
//                 typedef typename PairBlockJacob::first_type ParamType;
//                 auto& block = pair_block_jacob.second;
//                 auto& container_delta = bf::at_key<ParamType>(ba_.delta);
//                 typedef typename std::decay<decltype(container_delta)>::type::MatrixTag MatrixTag;
//                 const auto& map_indice = bundle_.spi2.indices(ttt::Indice<Obs>(iobs));// vector<Indice,...>
//                 auto& indice = *bf::find<ttt::Indice<ParamType>>(map_indice);
// 
//                 pred += 
//                         squared_norm(make_view(map_erreur.template at_key<Obs>().at(iobs).first,ttt::wrap<MatrixTag>())
//                         +
//                         block * container_delta(indice)
//                         + vec);
//               });
//           }
//         });
//       // now I'm damned
//       return pred;
//     }
    
//     void compute(Bundle& bundle_, double& lambda, bool recalcul, double& pred)
//     {
//       set_zero_(ba_);
//       Policy::init_zero();
// 
//       if (recalcul)
//       {
//         bf::for_each(ba_.h,detail::SetZero());
//         bf::for_each(ba_.jte,detail::SetZero());
//         // compute H = JtJ & Jte
//         detail::fill_hessien_residu33<Derivator<typename Policy::MatrixTag>>(bundle_,ba_,map_erreur);
//         Policy::save_h(ba_.h);
//       }
//       else
//       {
//         Policy::reload_h(ba_.h,-prev_lambda);
//       }
// 
//       prev_lambda = lambda;
//       for_each<MetaUnary<typename Ba::ListeDiag>>(std::tie(ba_.h),LambdaDiag(lambda));
//       Policy::solve(ba_,bundle_);
//       
//       pred = prediction(bundle_);
//       
//       for_each<MetaBinary<typename Ba::Keys>>(std::tie(bundle_.opt_container.map(),ba_.delta),Correct());
//     }
    
    void compute(Bundle& bundle_, double& lambda, bool recalcul)
    {
      Tic tic("compute all");
      assert(bundle_.nb_obs()!=0);
      try
      {
        set_zero_(ba_);
        Policy::init_zero();

        free_tic.tic();
        Tic tic_h("compute H");
        if (recalcul)
        {
          bf::for_each(ba_.h,detail::SetZero());
          bf::for_each(ba_.jte,detail::SetZero());
          // compute H = JtJ & Jte
          detail::fill_hessien<typename Policy::MatrixTag>(bundle_,ba_,map_erreur,meds);
          Policy::save_h(ba_.h);
        }
        else
        {
          Policy::reload_h(ba_.h,-prev_lambda);
        }
        tic_h.disp();
        jacobian_evaluations += free_tic.toc();

        free_tic.tic();

        
        // H += Eye(H)*lambda
        if (lambda==-1)
        {
          lambda = init_lambda();
        }

        prev_lambda = lambda;
        for_each<MetaUnary<typename Ba::ListeDiag>>(std::tie(ba_.h),LambdaDiag(lambda));
        Tic tic_schur("SolveDelta");


//         clement(ba_.h);
//         std::cout << ba_.h << std::endl;
//           auto a = to_mat<typename Ba::Keys,typename Ba::ListeHessien>(ba_.h,size_tuple<mpl::size<typename Ba::Keys>::value>(ba_.delta));
          
//         auto u = Blocker<2,2>::view(a,0,0);
//         auto v = Blocker<2,2>::view(a,2,2);
//         auto w = Blocker<2,2>::view(a,0,2);
//         
//         std::cout << " A :\n" << a << std::endl << std::endl << std::endl;
//         std::cout << " V :\n" << v << std::endl;
//         std::cout << " W :\n" << w << std::endl;
//         
//         auto b = to_matv(ba_.jte());
//         auto ea = Blocker<2,1>::view(b,0,0);
//         auto eb = Blocker<2,1>::view(b,2,0);
//         std::cout << " jte : " << b.transpose() << std::endl;
//         std::cout << " ea  : " << ea.transpose() << std::endl;
//         std::cout << " eb  : " << eb.transpose() << std::endl;
//         
//         auto y = w*v.inverse();
//         auto s = u - y * w.transpose();
//         auto e = ea - y * eb;
//         
//         auto da = (s.template selfadjointView<Eigen::Upper>().llt().solve(e)).eval();
//         auto db = v.inverse()*(eb - w.transpose() * da);
//         
//         std::cout << " S : \n" << s << std::endl;
//         std::cout << std::endl;
//         
//         std::cout << da.transpose() << " " << db.transpose() << std::endl;
        
        Policy::solve(ba_,bundle_);
        
//         std::cout << to_matv(ba_.delta()).transpose() << std::endl;
//         std::cout << std::endl;
        tic_schur.disp();
        
        for_each<MetaBinary<typename Ba::Keys>>(std::tie(bundle_.opt_container.map(),ba_.delta),Correct());
        
        norm_eq_ += free_tic.toc();
      }
      catch(NAN_ERROR& e)
      {
        std::cout << " Optimization failure : " <<  e.what() << std::endl;
      }
      tic.disp();
    }
  };
}

namespace ttt
{
  template<class A> struct Name< lma::LevMar<A> > { static std::string name(){ return "LevMar<" + ttt::name<A>() + ">";} };
}

#endif

