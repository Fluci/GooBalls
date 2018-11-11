#include "neighborhood.hpp"

namespace GooBalls {

namespace Spatial {


std::vector<std::vector<Neighborhood::Index>>& Neighborhood::indexes(){
    return m_indexes;
}

const std::vector<std::vector<Neighborhood::Index>>& Neighborhood::indexes() const {
    return m_indexes;
}

void Neighborhood::resetIndexes(int size){
    for(auto& l : m_indexes){
        l.clear();
    }
    m_indexes.resize(size);
}

} // Spatial

} // GooBalls
