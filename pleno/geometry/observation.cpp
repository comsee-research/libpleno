#include "observation.h"

#include <iostream>
#include "types.h"

//******************************************************************************
//******************************************************************************
std::ostream& operator<<(std::ostream& os, const MicroImageCenterObservation& ob)
{	
	os 	<< "{" << std::endl
		<< "(k,l) = (" << ob.k <<", " << ob.l << ")" << std::endl
		<< "(u,v) = (" << ob[0] <<", " << ob[1] << ")" << std::endl
		<< "};" ;
		
	return os;
}

void save(v::OutputArchive& archive, const MicroImageCenterObservation& m)
{
	archive
		("k", m.k)
		("l", m.l)
		("u", m.u)
		("v", m.v);
}

void load(v::InputArchive& archive, MicroImageCenterObservation& m)
{
	archive
		("k", m.k)
		("l", m.l)
		("u", m.u)
		("v", m.v);
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void save(v::OutputArchive& archive, const CheckerBoardObservation& o)
{
    archive
    ("k", o.k)
    ("l", o.l)
    ("u", o.u)
    ("v", o.v)
    ("cluster", o.cluster)
    ("frame", o.frame);
}

void load(v::InputArchive& archive, CheckerBoardObservation& o)
{
    archive
    ("k", o.k)
    ("l", o.l)
    ("u", o.u)
    ("v", o.v)
    ("cluster", o.cluster)
    ("frame", o.frame);
}

std::ostream& operator<<(std::ostream& os, const CheckerBoardObservation& ob)
{	
	os 	<< "{" << std::endl
		<< "(k,l) = (" << ob.k <<", " << ob.l << ")" << std::endl
		<< "(u,v) = (" << ob[0] <<", " << ob[1] << ")" << std::endl
		<< "(c,f) = (" << ob.cluster <<", " << ob.frame << ")" << std::endl
		<< "};" ;
		
	return os;
}

CheckerBoardObservation::operator BlurAwarePlenopticObservation() const 
{ 
	return BlurAwarePlenopticObservation{k,l,u,v,-1.0,cluster,frame,isValid}; 
}

//******************************************************************************
//******************************************************************************
//******************************************************************************
void save(v::OutputArchive& archive, const BlurAwarePlenopticObservation& o)
{
    archive
    ("k", o.k)
    ("l", o.l)
    ("u", o.u)
    ("v", o.v)
    ("rho", o.rho)
    ("cluster", o.cluster)
    ("frame", o.frame);
}

void load(v::InputArchive& archive, BlurAwarePlenopticObservation& o)
{
    archive
    ("k", o.k)
    ("l", o.l)
    ("u", o.u)
    ("v", o.v)
    ("rho", o.rho)
    ("cluster", o.cluster)
    ("frame", o.frame);
}

std::ostream& operator<<(std::ostream& os, const BlurAwarePlenopticObservation& ob)
{	
	os 	<< "{" << std::endl
		<< "(k,l) = (" << ob.k <<", " << ob.l << ")" << std::endl
		<< "(u,v,rho) = (" << ob[0] <<", " << ob[1] <<", " << ob[2]  << ")" << std::endl
		<< "(c,f) = (" << ob.cluster <<", " << ob.frame << ")" << std::endl
		<< "};" ;
		
	return os;
}

BlurAwarePlenopticObservation::operator CheckerBoardObservation() const 
{ 
	return CheckerBoardObservation{k,l,u,v,cluster,frame,isValid}; 
}
