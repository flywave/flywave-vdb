#include "texture2d.hh"

namespace flywave {
template class texture2d<uint8_t>;
template class texture2d<color2<uint8_t>>;
template class texture2d<color3<uint8_t>>;
template class texture2d<color4<uint8_t>>;

template class texture2d<uint16_t>;
template class texture2d<color2<uint16_t>>;
template class texture2d<color3<uint16_t>>;
template class texture2d<color4<uint16_t>>;

template class texture2d<float>;
template class texture2d<color2<float>>;
template class texture2d<color3<float>>;
template class texture2d<color4<float>>;

template class texture2d<double>;
template class texture2d<color2<double>>;
template class texture2d<color3<double>>;
template class texture2d<color4<double>>;
} // namespace flywave
