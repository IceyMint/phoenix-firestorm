/** 
 * @file llstl.h
 * @brief helper object & functions for use with the stl.
 *
 * $LicenseInfo:firstyear=2003&license=viewerlgpl$
 * Second Life Viewer Source Code
 * Copyright (C) 2010, Linden Research, Inc.
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation;
 * version 2.1 of the License only.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 * 
 * Linden Research, Inc., 945 Battery Street, San Francisco, CA  94111  USA
 * $/LicenseInfo$
 */

#ifndef LL_LLSTL_H
#define LL_LLSTL_H

#include "stdtypes.h"
#include <functional>
#include <algorithm>
#include <map>
#include <vector>
#include <list>
#include <set>
#include <typeinfo>

#ifdef LL_LINUX
// <ND> For strcmp
#include <string.h>
#endif

// Use to compare the contents of two pointers (e.g. std::string*)
template <typename T>
struct compare_pointer_contents
{
	typedef const T* Tptr;
	bool operator()(const Tptr& a, const Tptr& b) const
	{
		return *a < *b;
	}
};

// DeletePointer is a simple helper for deleting all pointers in a container.
// The general form is:
//
//  std::for_each(cont.begin(), cont.end(), DeletePointer());
//  somemap.clear();
//
// Don't forget to clear()!

struct DeletePointer
{
	template<typename T> void operator()(T* ptr) const
	{
		delete ptr;
	}
};
struct DeletePointerArray
{
	template<typename T> void operator()(T* ptr) const
	{
		delete[] ptr;
	}
};

// DeletePairedPointer is a simple helper for deleting all pointers in a map.
// The general form is:
//
//  std::for_each(somemap.begin(), somemap.end(), DeletePairedPointer());
//  somemap.clear();		// Don't leave dangling pointers around

struct DeletePairedPointer
{
	template<typename T> void operator()(T &ptr) const
	{
		delete ptr.second;
		ptr.second = NULL;
	}
};
struct DeletePairedPointerArray
{
	template<typename T> void operator()(T &ptr) const
	{
		delete[] ptr.second;
		ptr.second = NULL;
	}
};


template<typename T, typename ALLOC>
void delete_and_clear(std::list<T*, ALLOC>& list)
{
	std::for_each(list.begin(), list.end(), DeletePointer());
	list.clear();
}

template<typename T, typename ALLOC>
void delete_and_clear(std::vector<T*, ALLOC>& vector)
{
	std::for_each(vector.begin(), vector.end(), DeletePointer());
	vector.clear();
}

template<typename T, typename COMPARE, typename ALLOC>
void delete_and_clear(std::set<T*, COMPARE, ALLOC>& set)
{
	std::for_each(set.begin(), set.end(), DeletePointer());
	set.clear();
}

template<typename K, typename V, typename COMPARE, typename ALLOC>
void delete_and_clear(std::map<K, V*, COMPARE, ALLOC>& map)
{
	std::for_each(map.begin(), map.end(), DeletePairedPointer());
	map.clear();
}

template<typename T>
void delete_and_clear(T*& ptr)
{
	delete ptr;
	ptr = NULL;
}


template<typename T>
void delete_and_clear_array(T*& ptr)
{
	delete[] ptr;
	ptr = NULL;
}

// Simple function to help with finding pointers in maps.
// For example:
// 	typedef  map_t;
//  std::map<int, const char*> foo;
//	foo[18] = "there";
//	foo[2] = "hello";
// 	const char* bar = get_ptr_in_map(foo, 2); // bar -> "hello"
//  const char* baz = get_ptr_in_map(foo, 3); // baz == NULL
template <typename K, typename T>
inline T* get_ptr_in_map(const std::map<K,T*>& inmap, const K& key)
{
	// Typedef here avoids warnings because of new c++ naming rules.
	typedef typename std::map<K,T*>::const_iterator map_iter;
	map_iter iter = inmap.find(key);
	if(iter == inmap.end())
	{
		return NULL;
	}
	else
	{
		return iter->second;
	}
};

// helper function which returns true if key is in inmap.
template <typename K, typename T>
inline bool is_in_map(const std::map<K,T>& inmap, const K& key)
{
	if(inmap.find(key) == inmap.end())
	{
		return false;
	}
	else
	{
		return true;
	}
}

// Similar to get_ptr_in_map, but for any type with a valid T(0) constructor.
// To replace LLSkipMap getIfThere, use:
//   get_if_there(map, key, 0)
// WARNING: Make sure default_value (generally 0) is not a valid map entry!
template <typename K, typename T>
inline T get_if_there(const std::map<K,T>& inmap, const K& key, T default_value)
{
	// Typedef here avoids warnings because of new c++ naming rules.
	typedef typename std::map<K,T>::const_iterator map_iter;
	map_iter iter = inmap.find(key);
	if(iter == inmap.end())
	{
		return default_value;
	}
	else
	{
		return iter->second;
	}
};

// Useful for replacing the removeObj() functionality of LLDynamicArray
// Example:
//  for (std::vector<T>::iterator iter = mList.begin(); iter != mList.end(); )
//  {
//    if ((*iter)->isMarkedForRemoval())
//      iter = vector_replace_with_last(mList, iter);
//    else
//      ++iter;
//  }
template <typename T>
inline typename std::vector<T>::iterator vector_replace_with_last(std::vector<T>& invec, typename std::vector<T>::iterator iter)
{
	typename std::vector<T>::iterator last = invec.end(); --last;
	if (iter == invec.end())
	{
		return iter;
	}
	else if (iter == last)
	{
		invec.pop_back();
		return invec.end();
	}
	else
	{
		*iter = *last;
		invec.pop_back();
		return iter;
	}
};

// Example:
//   vector_replace_with_last(mList, x);
template <typename T>
inline bool vector_replace_with_last(std::vector<T>& invec, const T& val)
{
	typename std::vector<T>::iterator iter = std::find(invec.begin(), invec.end(), val);
	if (iter != invec.end())
	{
		typename std::vector<T>::iterator last = invec.end(); --last;
		*iter = *last;
		invec.pop_back();
		return true;
	}
	return false;
}

// Append N elements to the vector and return a pointer to the first new element.
template <typename T>
inline T* vector_append(std::vector<T>& invec, S32 N)
{
	U32 sz = invec.size();
	invec.resize(sz+N);
	return &(invec[sz]);
}

// call function f to n members starting at first. similar to std::for_each
template <class InputIter, class Size, class Function>
Function ll_for_n(InputIter first, Size n, Function f)
{
	for ( ; n > 0; --n, ++first)
		f(*first);
	return f;
}

// copy first to result n times, incrementing each as we go
template <class InputIter, class Size, class OutputIter>
OutputIter ll_copy_n(InputIter first, Size n, OutputIter result)
{
	for ( ; n > 0; --n, ++result, ++first)
		*result = *first;
	return result;
}

// set  *result = op(*f) for n elements of f
template <class InputIter, class OutputIter, class Size, class UnaryOp>
OutputIter ll_transform_n(
	InputIter first,
	Size n,
	OutputIter result,
	UnaryOp op)
{
	for ( ; n > 0; --n, ++result, ++first)
		*result = op(*first);
	return result;
}


/**
 * Compare std::type_info* pointers a la std::less. We break this out as a
 * separate function for use in two different std::less specializations.
 */
inline
bool before(const std::type_info* lhs, const std::type_info* rhs)
{
#if LL_LINUX && defined(__GNUC__) && ((__GNUC__ < 4) || (__GNUC__ == 4 && __GNUC_MINOR__ < 4))
    // If we're building on Linux with gcc, and it's either gcc 3.x or
    // 4.{0,1,2,3}, then we have to use a workaround. Note that we use gcc on
    // Mac too, and some people build with gcc on Windows (cygwin or mingw).
    // On Linux, different load modules may produce different type_info*
    // pointers for the same type. Have to compare name strings to get good
    // results.
    return strcmp(lhs->name(), rhs->name()) < 0;
#else  // not Linux, or gcc 4.4+
    // Just use before(), as we normally would
    return lhs->before(*rhs) ? true : false;
#endif
}

/**
 * Specialize std::less<std::type_info*> to use std::type_info::before().
 * See MAINT-1175. It is NEVER a good idea to directly compare std::type_info*
 * because, on Linux, you might get different std::type_info* pointers for the
 * same type (from different load modules)!
 */
namespace std
{
	template <>
	struct less<const std::type_info*>
	{
		bool operator()(const std::type_info* lhs, const std::type_info* rhs) const
		{
			return before(lhs, rhs);
		}
	};

	template <>
	struct less<std::type_info*>
	{
		bool operator()(std::type_info* lhs, std::type_info* rhs) const
		{
			return before(lhs, rhs);
		}
	};
} // std


/**
 * Implementation for ll_template_cast() (q.v.).
 *
 * Default implementation: trying to cast two completely unrelated types
 * returns 0. Typically you'd specify T and U as pointer types, but in fact T
 * can be any type that can be initialized with 0.
 */
template <typename T, typename U>
struct ll_template_cast_impl
{
    T operator()(U)
    {
        return 0;
    }
};

/**
 * ll_template_cast<T>(some_value) is for use in a template function when
 * some_value might be of arbitrary type, but you want to recognize type T
 * specially.
 *
 * It's designed for use with pointer types. Example:
 * @code
 * struct SpecialClass
 * {
 *     void someMethod(const std::string&) const;
 * };
 *
 * template <class REALCLASS>
 * void somefunc(const REALCLASS& instance)
 * {
 *     const SpecialClass* ptr = ll_template_cast<const SpecialClass*>(&instance);
 *     if (ptr)
 *     {
 *         ptr->someMethod("Call method only available on SpecialClass");
 *     }
 * }
 * @endcode
 *
 * Why is this better than dynamic_cast<>? Because unless OtherClass is
 * polymorphic, the following won't even compile (gcc 4.0.1):
 * @code
 * OtherClass other;
 * SpecialClass* ptr = dynamic_cast<SpecialClass*>(&other);
 * @endcode
 * to say nothing of this:
 * @code
 * void function(int);
 * SpecialClass* ptr = dynamic_cast<SpecialClass*>(&function);
 * @endcode
 * ll_template_cast handles these kinds of cases by returning 0.
 */
template <typename T, typename U>
T ll_template_cast(U value)
{
    return ll_template_cast_impl<T, U>()(value);
}

/**
 * Implementation for ll_template_cast() (q.v.).
 *
 * Implementation for identical types: return same value.
 */
template <typename T>
struct ll_template_cast_impl<T, T>
{
    T operator()(T value)
    {
        return value;
    }
};

/**
 * LL_TEMPLATE_CONVERTIBLE(dest, source) asserts that, for a value @c s of
 * type @c source, <tt>ll_template_cast<dest>(s)</tt> will return @c s --
 * presuming that @c source can be converted to @c dest by the normal rules of
 * C++.
 *
 * By default, <tt>ll_template_cast<dest>(s)</tt> will return 0 unless @c s's
 * type is literally identical to @c dest. (This is because of the
 * straightforward application of template specialization rules.) That can
 * lead to surprising results, e.g.:
 *
 * @code
 * Foo myFoo;
 * const Foo* fooptr = ll_template_cast<const Foo*>(&myFoo);
 * @endcode
 *
 * Here @c fooptr will be 0 because <tt>&myFoo</tt> is of type <tt>Foo*</tt>
 * -- @em not <tt>const Foo*</tt>. (Declaring <tt>const Foo myFoo;</tt> would
 * force the compiler to do the right thing.)
 *
 * More disappointingly:
 * @code
 * struct Base {};
 * struct Subclass: public Base {};
 * Subclass object;
 * Base* ptr = ll_template_cast<Base*>(&object);
 * @endcode
 *
 * Here @c ptr will be 0 because <tt>&object</tt> is of type
 * <tt>Subclass*</tt> rather than <tt>Base*</tt>. We @em want this cast to
 * succeed, but without our help ll_template_cast can't recognize it.
 *
 * The following would suffice:
 * @code
 * LL_TEMPLATE_CONVERTIBLE(Base*, Subclass*);
 * ...
 * Base* ptr = ll_template_cast<Base*>(&object);
 * @endcode
 *
 * However, as noted earlier, this is easily fooled:
 * @code
 * const Base* ptr = ll_template_cast<const Base*>(&object);
 * @endcode
 * would still produce 0 because we haven't yet seen:
 * @code
 * LL_TEMPLATE_CONVERTIBLE(const Base*, Subclass*);
 * @endcode
 *
 * @TODO
 * This macro should use Boost type_traits facilities for stripping and
 * re-adding @c const and @c volatile qualifiers so that invoking
 * LL_TEMPLATE_CONVERTIBLE(dest, source) will automatically generate all
 * permitted permutations. It's really not fair to the coder to require
 * separate:
 * @code
 * LL_TEMPLATE_CONVERTIBLE(Base*, Subclass*);
 * LL_TEMPLATE_CONVERTIBLE(const Base*, Subclass*);
 * LL_TEMPLATE_CONVERTIBLE(const Base*, const Subclass*);
 * @endcode
 *
 * (Naturally we omit <tt>LL_TEMPLATE_CONVERTIBLE(Base*, const Subclass*)</tt>
 * because that's not permitted by normal C++ assignment anyway.)
 */
#define LL_TEMPLATE_CONVERTIBLE(DEST, SOURCE)   \
template <>                                     \
struct ll_template_cast_impl<DEST, SOURCE>      \
{                                               \
    DEST operator()(SOURCE wrapper)             \
    {                                           \
        return wrapper;                         \
    }                                           \
}


#endif // LL_LLSTL_H
