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

#ifndef __OPTIMISATION2_BA_CONTAINER_HPP__
#define __OPTIMISATION2_BA_CONTAINER_HPP__

#include "../container/container.hpp"

#include <libv/lma/color/console.hpp>
#include <cassert>
#include <libv/core/logger.hpp>

#include <libv/lma/ttt/traits/wrap.hpp>
#include <libv/lma/ttt/traits/typed_indice.hpp>
#include <libv/lma/lm/bundle/indice_container.hpp>
#include <libv/lma/lm/trait/size.hpp>

#include <libv/lma/string/display.hpp>
#include <boost/fusion/include/make_vector.hpp>//! pour la liste computeS
#include "mat.hpp"

#include <type_traits>

namespace lma {

  template<class Id, class Tag_, bool column> class Vector
  {
    public:
      const static std::size_t I = Size<Id>::value;
      const static std::size_t J = (column?1:I);
      typedef Tag_ MatrixTag;
      typedef ContainerOption<MatrixTag,I,J> MatrixMaker;
      typedef typename MatrixMaker::Matrix Matrix;
      typedef ttt::Indice<Id> Indice;
      typedef Id Key;
      typedef typename MatrixMaker::Float Float;
      
      typedef Vector<Id,MatrixTag,column> self;

      Vector(){/* std::cout << name() << std::endl;*/ }

      static std::string name() 
      { return color.cyan() + "Vector<" + color.reset() + ttt::name<Id>() + "," + ttt::name<Matrix>() + color.cyan() + ">" +color.reset(); }
      
      bool empty() const { return v.empty(); }
      void resize(const Indice& indice) { v.resize(indice(),MatrixMaker::Zero()); }
      void assign(const Indice& indice) { v.assign(indice(),MatrixMaker::Zero()); }

      Matrix& operator()(const Indice& indice) {
        assert(indice<size());return v[indice()]; }
      const Matrix& operator()(const Indice& indice) const {
        assert(indice<size());return v[indice()]; }

      Indice first() const { return Indice(0); }
      Indice size() const { return Indice(v.size()); }
      size_t full_size() const { return v.size() * I; }

      void set_zero() { for(auto& x : v) x = MatrixMaker::Zero();}

      bool is_invalid() const
      {
        for(auto& x : v)
          for(size_t k = 0 ; k < I ; ++k)
          {
            if (lma::is_invalid(x[k])) 
            {
              std::cout << " invalid value : " << x[k] << std::endl;
              return true;
            }
          }
        return false;
      }
      
      typedef std::vector<Matrix,Eigen::aligned_allocator<Matrix>> TVector;
      
      void disp() const
      {
        for(auto& x : v)
          std::cout << x.transpose() << " ";
        std::cout << std::endl;
      }

      void copy_mat(const self& vector)
      {
        v = vector.v;
      }

      TVector v;   

    public:

      typedef typename TVector::iterator iterator;
      typedef typename TVector::const_iterator citerator;
      iterator begin() { return v.begin(); }
      iterator end() { return v.end(); }
      citerator begin() const { return v.cbegin(); }
      citerator end() const { return v.cend(); }
  };



  template<class Id, class Tag_> class VectorColumn : public Vector<Id,Tag_,true>
  {
    public:
      typedef Tag_ Tag;
      typedef Vector<Id,Tag,true> parent;
      using parent::I;
      typedef typename parent::Matrix Matrix;
      typedef typename parent::Float Float;
      using parent::first;
      using parent::size;
      using parent::empty;
//       Vector(){/* std::cout << name() << std::endl;*/ }

      static std::string name() 
      { return color.cyan() + "VectorColumn<" + color.reset() + ttt::name<Id>() + "," + ttt::name<Matrix>() + color.cyan() + ">" +color.reset(); }
      
      void minus() { for(auto& x : *this) x = -x; }

      Float squaredNorm() const
      {
        Float n = 0;
        for(auto& x : *this)
          n += squared_norm(x);
        return n;
      }
      
//       Float norm() const
//       {
//         Float n = 0;
//         for(auto& x : *this)
//           n += std::sqrt(squared_norm(x));
//         return n;
//       }

      Float compute_dot(const VectorColumn<Id,Tag>& b) const
      {
        Float res = 0;
        assert(b.size()==size());
        for(auto i = first() ; i < size() ; ++i)
          res += dot((*this)(i),b(i));
        return res;
      }
  };

  template<class Id, class T>
  std::ostream& operator<<(std::ostream& o, const VectorColumn<Id,T>& residu)
  {
    o << color.green() << residu.name() << color.reset() << std::endl;

    for(auto i = residu.first() ; i < residu.size() ; ++i)
    {
      if (i==0) std::cout << color.cyan() << " ==== " << i << " <" << ttt::name<decltype(i)>() << "> ==== " << color.reset();
      else
        std::cout << color.cyan() << " ==== " << i << color.reset() ;
      o << std::endl << residu(i) << std::endl;
    }
    return o;
  }


  /// ########### Table #############

  
  template<class Id1_, class Id2_, class Tag, class Spec = void> struct Table
  {
      typedef Id1_ Id1;
      typedef Id2_ Id2;
      typedef Table<Id1,Id2,Tag,Spec> self;
      typedef Id1 P1;
      typedef Id2 P2;
      static const size_t I = Size<Id1>::value;
      static const size_t J = Size<Id2>::value;
      typedef ContainerOption<Tag,I,J> MatrixMaker;
      typedef typename MatrixMaker::Matrix Matrix;
//       typedef typename MatrixMaker::MatrixDD MatrixDD;
      typedef typename MatrixMaker::Float Float;
      typedef Id1 Key1;
      typedef Id2 Key2;
      typedef SIC<Id1,Id2> IndiceContainer;
      
      typedef typename IndiceContainer::IndiceA Indice1;
      typedef typename IndiceContainer::SparseAB tSparse;

      typedef std::vector< Matrix, Eigen::aligned_allocator<Matrix>> TVector;
      TVector v;
      std::vector<int> voffset;
      IndiceContainer indice;
      
      static std::size_t H() { return I; }
      static std::size_t W() { return J; }
      static std::string name() { return color.cyan() + "Table<" + color.reset() + ttt::name<Id1>() + "," + ttt::name<Id2>() + " : " + ttt::name<Matrix>() + color.cyan() + ">" + color.reset(); }

      Table() {}

      void set(const SIC<Id1,Id2>& cont_) { resize(cont_); }

      void resize(const SIC<Id1,Id2>& cont_)
      {
        indice = cont_;
        /*
        if (indice.size()==0) 
        {
          std::string msg = "Warning : indice.size()==0" + name();
          V_DUMP(msg);
        }
        */
        voffset.clear();
        std::size_t total = 0;
        for(auto i = indice.first() ; i < indice.size() ; ++i)
        {
          voffset.push_back(total);
          total += indice.size(i)();
        }
        v.assign(total, MatrixMaker::Zero());
      }


      //! réserver begin et end pour iterator -> for( auto x : table ) {...}
      auto first() const ->decltype(indice.first()) { return indice.first(); }
      auto size() const ->decltype(indice.size()) { return indice.size(); }

      auto first(const Indice1& indice1) const ->decltype(indice.first(indice1)) { return indice.first(indice1); }
      auto size(const Indice1& indice1) const ->decltype(indice.size(indice1)) { return indice.size(indice1); }

      Matrix& operator()(const Indice1& indice1, const tSparse& indice2)
      {
        assert(indice1 < voffset.size());
        assert(size_t(voffset[indice1()] + indice2()) < v.size());
        return v[voffset[indice1()] + indice2()];
      }

      const Matrix& operator()(const Indice1& indice1, const tSparse& indice2) const
      {
        assert(indice1 < voffset.size());
        if (!(size_t(voffset[indice1()] + indice2()) < v.size()))
        {
          std::cout << " voffset[indice1()] + indice2()) < v.size() " << std::endl;
          std::cout << " voffset["<<indice1() << "]=" << voffset[indice1()] << " + " << indice2() << "  )" << " <   " << v.size() << std::endl;
        }
        assert(size_t(voffset[indice1()] + indice2()) < v.size());
        return v[voffset[indice1()] + indice2()];
      }

      void disp() const { for(auto&x : v) std::cout << x << std::endl << std::endl; }

      void copy_mat(const Table<Id1,Id2,Tag>& table)
      {
        v = table.v;voffset = table.voffset;
      }

      
      bool is_invalid() const
      {
        for(auto& x : v)
          for(size_t k = 0 ; k < I ; ++k)
            if (is_invalid(x(k)))
            {
              std::cout << " Invalid value " << x(k) << std::endl;
              return true;
            }
        return false;
      }

      void set_zero() { for(auto& x : v) x = MatrixMaker::Zero();}

      typedef typename TVector::iterator iterator;
      typedef typename TVector::const_iterator citerator;
      iterator begin() { return v.begin(); }
      iterator end() { return v.end(); }
      citerator begin() const { return v.cbegin(); }
      citerator end() const { return v.cend(); }
  };

  
  struct Diagonal{};
  struct Symetric{};

  template<class P, class T_> struct Table<P,P,T_,Symetric> : public Table<P,P,T_>
  {
    typedef Table<P,P,T_> parent;
    typedef typename parent::MatrixMaker MatrixMaker;
    typedef T_ Tag;
    typedef P Id1;
    typedef P Id2;
    typedef typename parent::Float Float;
//     typedef typename parent::MatrixDD MatrixDD;
    typedef typename parent::Matrix Matrix;
    static const size_t I = parent::I;
    static const size_t J = parent::J;
    using parent::indice;
    using parent::first;
    using parent::size;
    using parent::v;
    using parent::voffset;

//     static std::string name() { return color.cyan() + "SymetricTable<" + color.reset() + ttt::name<Id1>() + "," + ttt::name<Id2>() + " : " + ttt::name<Matrix>() + color.cyan() + ">" + color.reset(); }
    static std::string name() { return std::string("SymetricTable::[") + parent::name() + "]"; }

//     SparseTable(){std::cout << name()  << std::endl;}

    void resize(const SIC<Id1,Id1>& cont_)
    {
      indice = cont_;
      /*
      if (indice.size()==0) 
      {
        std::string msg = "Warning : indice.size()==0" + name();
        V_DUMP(msg);
      }
      */
      voffset.clear();
      std::size_t total = 0;
      for(auto i = indice.first() ; i < indice.size() ; ++i)
      {
        voffset.push_back(total);
        total += indice.size(i)();
      }
      v.assign(total, MatrixMaker::Zero());
    }
      
    void inv()
    {
      
      for(auto i = first() ; i < size() ; ++i)
      {
        inverse_in_place(this->operator()(i,0));
//         assert( x != Matrix::Zero() );
//         this->operator()(i,0) = this->operator()(i,0).inverse().eval();
            
//         if (std::isinf(x(0))||std::isnan(x(0)))
//           throw ttt::Indice<Id1>(&x - &v[0]);
//         assert( !std::isinf(x(0)) );
      }
      
    }

    Float max_lambda() const
    {
      Float lambda = 0;
      for(auto i = first() ; i < size() ; ++i)
        for(size_t k = 0 ; k < I; ++k)
          lambda = std::max(std::abs(at((*this)(i,0),k,k)),lambda);
      return lambda;
    }
    
    void augment_diag(double lambda)
    {
      for(auto i = first() ; i < size() ; ++i)
//         damp_diag(this->operator()(i,0),lambda);
	for(size_t k = 0 ; k < I ; ++k)
	  at((*this)(i,0),k,k) +=lambda;
    }
    
    void set_diag_inv(const Table<Id1,Id1,Tag,Symetric>& table)
    {
      voffset.clear();
      v.clear();
      std::size_t total = 0;
      indice.set_max(table.size(),table.size());
      for(auto i = table.first() ; i < table.size() ; ++i)
      {
        indice.add(i,i);
        voffset.push_back(total);
        total++;
      }
      v.resize(total);//, Matrix::Zero());
      for(auto i = table.first() ; i < table.size() ; ++i)
        v[i()] = inverse(table(i,0));
    }
    
    void set_diag_inv(const Table<Id1,Id1,Tag,Diagonal>& table)
    {
      voffset.clear();
      v.clear();
      std::size_t total = 0;
      indice.set_max(table.size(),table.size());
      for(auto i = table.first() ; i < table.size() ; ++i)
      {
        indice.add(i,i);
        voffset.push_back(total);
        total++;
      }
      v.resize(total);//, MatrixMaker::Zero());
      for(auto i = table.first() ; i < table.size() ; ++i)
        v[i()] = inverse(table(i));
    }

    void copy_mat(const Table<Id1,Id1,Tag,Diagonal>& table)
    {
      voffset.clear();
      v.clear();
      std::size_t total = 0;
      indice.set_max(table.size(),table.size());
      for(auto i = table.first() ; i < table.size() ; ++i)
      {
        indice.add(i,i);
        voffset.push_back(total);
        total++;
      }
      v.resize(total);//, MatrixMaker::Zero());
      for(auto i = table.first() ; i < table.size() ; ++i)
        v[i()] = table(i);
    }
  };
  
  template<class P, class Q, class T> struct Table<P,Q,T,Diagonal> : public Vector<P,T,false>
  {
    static_assert(boost::is_same<P,Q>::value,"Table<P,Q>: P!=Q");
    typedef P Id1;
    typedef P Id2;
    typedef Vector<P,T,false> parent;
    typedef Table<P,P,T,Diagonal> self;
    typedef typename parent::Float Float;
    static std::string name() { return std::string("DiagonalTable::[") + parent::name() + "]"; }
    
    using parent::I;
    
    void augment_diag(double lambda)
    {
      for(auto& block : *this)
        for(size_t k = 0 ; k < I ; ++k)
          at(block,k,k) += lambda;
    }
    
    void resize(const SIC<P,P>& sic)
    {
      parent::resize(sic.size());
    }
    
    Float max_lambda() const
    {
      Float lambda = 0;
      for(auto& x : *this)
        for(size_t k = 0 ; k < I; ++k)
          lambda = std::max(std::abs(at(x,k,k)),lambda);
      return lambda;
    }
    
    void inv()
    {
      for(auto& x : *this)
      {
        inverse_in_place(x);
        if (std::isinf(x(0,0)))
          throw ttt::Indice<P>( &x - &parent::v[0] );
        assert( !std::isinf(x(0,0)) );
      }
    }
  };
  
  template<class> struct IsDiagonal { static const bool value = false; };
  template<class P, class T> struct IsDiagonal<Table<P,P,T,Diagonal>> { static const bool value = true; };
  
  template<class M, class N, class T, class _> 
  std::ostream& disp_container(std::ostream& o, const Table<M,N,T,_>& inter, typename boost::disable_if<boost::is_same<_,Diagonal>>::type* =0)
  {
    o << color.green() << inter.name() << color.reset() << std::endl;
    for(auto i = inter.first(); i < inter.size(); ++i)
    {
      std::cout << color.cyan() << " ==== " << i << " <" << ttt::name<decltype(i)>() << "> ==== " << color.reset();
      for(auto j = inter.first(i) ; j < inter.size(i) ; ++j)
        if (j()==0) o << std::endl << color.magenta() << "   -- " << j << " <" << ttt::name<decltype(j)>() << "> -- \n" << color.reset() << inter(i,j) << std::endl;
        else
          o << std::endl << color.magenta() << "   -- " << j << std::endl << color.reset() << inter(i,j) << std::endl;
      if(inter.size(i)==0) std::cout << std::endl;
    }
    return o;
  }

  template<class M, class T>
  std::ostream& disp_container(std::ostream& o, const Table<M,M,T,Diagonal>& inter)
  {
    o << color.green() << inter.name() << color.reset() << std::endl;
    for(auto i = inter.first(); i < inter.size(); ++i)
    {
      std::cout << color.cyan() << " ==== " << i << " <" << ttt::name<decltype(i)>() << "> ==== " << color.reset();
      o << std::endl << color.magenta() << " -- \n" << color.reset() << inter(i) << std::endl;
      if(inter.size()==0) std::cout << std::endl;
    }
    return o;
  }
  
  template<class M, class N, class T, class _> std::ostream& operator<<(std::ostream& o, const Table<M,N,T,_>& table)
  {
    return disp_container(o,table);
  }
  
  

  template<class P, class T> using DiagonalTable = Table<P,P,T,Diagonal>;
  template<class P, class T> using SymetricTable = Table<P,P,T,Symetric>;

}// eon


namespace ttt
{
  template<class Id,class T, bool col> struct Name< lma::Vector<Id,T,col> > { static std::string name(){ return lma::Vector<Id,T,col>::name();} };
  template<class Id,class T> struct Name< lma::VectorColumn<Id,T> > { static std::string name(){ return lma::VectorColumn<Id,T>::name();} };
  
  template<class A, class B, class T> struct Name< lma::Table<A,B,T> > { static std::string name(){ return lma::Table<A,B,T>::name();} };
  template<class A, class T> struct Name< lma::SymetricTable<A,T> > { static std::string name(){ return lma::SymetricTable<A,T>::name();} };
  template<class A,class T> struct Name< lma::DiagonalTable<A,T> > { static std::string name(){ return lma::DiagonalTable<A,T>::name();} };
}

#include <libv/lma/lm/ba/tuple_to_mat.hpp>

#endif
