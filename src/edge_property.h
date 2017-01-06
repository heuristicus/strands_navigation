#ifndef EDGE_PROPERTY_H
#define EDGE_PROPERTY_H

#include "rviz/properties/property.h"

namespace rviz_topmap
{

/** @brief Property specialized to provide getter for booleans. */
class EdgeProperty: public rviz::Property
{
Q_OBJECT
public:
  EdgeProperty( const QString& name = QString(),
                bool default_value = false,
                const QString& description = QString(),
                Property* parent = 0,
                const char *changed_slot = 0,
                QObject* receiver = 0 );

  virtual ~EdgeProperty();

  virtual bool getBool() const;

public Q_SLOTS:
  bool setBool( bool value ) { return setValue( value ); }

};

} // end namespace rviz_topmap

#endif // EDGE_PROPERTY_H
