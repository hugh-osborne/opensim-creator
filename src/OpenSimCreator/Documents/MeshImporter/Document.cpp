#include "Document.hpp"

#include <OpenSimCreator/Documents/MeshImporter/Ground.hpp>
#include <OpenSimCreator/Documents/MeshImporter/MIIDs.hpp>

#include <oscar/Utils/ClonePtr.hpp>

osc::mi::Document::Document() :
    m_Objects{{MIIDs::Ground(), ClonePtr<MIObject>{Ground{}}}}
{
}
