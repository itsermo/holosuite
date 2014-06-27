#pragma once

#include <boost/shared_ptr.hpp>

namespace holo
{
	namespace filter
	{

		template <class T>
		class IHoloFilter
		{
		public:
			virtual ~IHoloFilter() = 0;
			virtual bool init() = 0;
			virtual void deinit() = 0;
			virtual bool isInit() = 0;

			virtual void doFilter(boost::shared_ptr<const T> inputFrame, boost::shared_ptr<T>& outputFrame) = 0;
		};
	}
}
