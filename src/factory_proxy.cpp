#include <m3rt/base/component.h>
#include <meka_omnibase_control/meka_omnibase_control.hpp>

///////////////////////////////////////////////////////
extern "C" 
{
///////////////////////////////////////////////////////
//These names should match the create_xxx() and destroy_xxx() function names.
//They should also match the names used for component definition in m3_config.yml 
#define BASE_TYPE_NAME	"meka_omnibase_control"

///////////////////////////////////////////////////////
//Creators
m3rt::M3Component * create_meka_omnibase_control(){return new meka_omnibase_control::MekaOmnibaseControl;}
//Deletors
void destroy_meka_omnibase_control(m3rt::M3Component* c) {delete c;}

///////////////////////////////////////////////////////
class M3FactoryProxy 
{ 
public:
	M3FactoryProxy()
	{
		m3rt::creator_factory[BASE_TYPE_NAME] =	create_meka_omnibase_control;
		m3rt::destroyer_factory[BASE_TYPE_NAME] =  destroy_meka_omnibase_control;
	}
};
///////////////////////////////////////////////////////
// The library's one instance of the proxy
M3FactoryProxy proxy;
}
