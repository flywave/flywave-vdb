#include "texture2d.hh"

namespace flywave {
template class texture2d<uint8_t>;
template class texture2d<vdb::math::Vec2<uint8_t>>;
template class texture2d<vdb::math::Vec3<uint8_t>>;
template class texture2d<vdb::math::Vec4<uint8_t>>;

template class texture2d<uint16_t>;
template class texture2d<vdb::math::Vec2<uint16_t>>;
template class texture2d<vdb::math::Vec3<uint16_t>>;
template class texture2d<vdb::math::Vec4<uint16_t>>;

template class texture2d<float>;
template class texture2d<vdb::math::Vec2<float>>;
template class texture2d<vdb::math::Vec3<float>>;
template class texture2d<vdb::math::Vec4<float>>;

template class texture2d<double>;
template class texture2d<vdb::math::Vec2<double>>;
template class texture2d<vdb::math::Vec3<double>>;
template class texture2d<vdb::math::Vec4<double>>;
} // namespace flywave
