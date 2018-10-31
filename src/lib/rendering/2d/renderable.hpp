#pragma once


namespace GooBalls {
namespace d2 {

/**
 * Abstract class that can render itself.
 * */
class Renderable {
public:
virtual void render() const = 0;

};

} // d2

} // GooBalls
