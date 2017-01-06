#include "edge_property.h"

#include <QColor>

namespace rviz_topmap
{

EdgeProperty::EdgeProperty(const QString& name,
			   bool default_value,
			   const QString& description,
			   Property* parent,
			   const char *changed_slot,
			   QObject* receiver)
  : rviz::Property(name, default_value, description, parent, changed_slot, receiver)
{
}

EdgeProperty::~EdgeProperty()
{
}

bool EdgeProperty::getBool() const
{
  return getValue().toBool();
}

} // end namespace rviz_topmap
