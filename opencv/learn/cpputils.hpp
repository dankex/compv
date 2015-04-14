/*
 * cpputils.hpp
 *
 *  Created on: Apr 14, 2015
 *      Author: danke
 */

#ifndef CPPUTILS_HPP_
#define CPPUTILS_HPP_

template <typename T, size_t N>
inline
size_t SizeOfArray( const T(&)[ N ] )
{
	return N;
}

#endif /* CPPUTILS_HPP_ */
