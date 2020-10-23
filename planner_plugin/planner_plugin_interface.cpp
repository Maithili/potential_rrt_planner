#include <openrave/plugin.h>
#include <boost/bind.hpp>

#include "PlannerModule.h"

using namespace OpenRAVE;

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "plannermodule" ) 
    {
        return InterfaceBasePtr(new PlannerModule(penv,sinput));
    }
    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Module].push_back("PlannerModule");   
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{}

