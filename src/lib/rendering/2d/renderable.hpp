#pragma once


namespace GooBalls {
namespace d2 {
namespace Render {

/**
 * Abstract class that can render itself.
 * */
class Renderable {
public:
virtual void render() const = 0;

};

} // Render

} // d2

} // GooBalls
