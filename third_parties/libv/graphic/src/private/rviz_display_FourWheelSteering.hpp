/**

\file
Des plugins rviz pour afficher des messages de type four_wheel_steering_msgs::FourWheelSteering ou four_wheel_steering_msgs::FourWheelSteeringStamped.

Les plugins sont exportés avec PLUGINLIB_EXPORT_CLASS dans ce fichier, et avec le fichier `rviz.xml` situé à la racine de ce package.

\author Alexis Wilhelm (2018)

*/

#include "TopicDisplay.hpp"
#include <libv/graphic/found/four_wheel_steering_msgs>
#if \
  defined LIBV_GRAPHIC_FOUR_WHEEL_STEERING_MSGS_FOUND &&\
  defined LIBV_GRAPHIC_RVIZ_FOUND &&\
  1
#include <four_wheel_steering_msgs/FourWheelSteeringStamped.h>
#include <pluginlib/class_list_macros.h>
#include <rviz/msg_conversions.h>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>

namespace v {
namespace graphic {
namespace {

/**
  Une flèche pour représenter une vitesse linéaire.
*/
struct Visual
{
  Visual(Ogre::SceneManager *manager, Ogre::SceneNode *node)
  : manager_(manager)
  , node_(node->createChildSceneNode())
  , head_(manager_, node_)
  , tail_(manager_, node_)
  {
    tail_.setHeadColor(Ogre::ColourValue::ZERO);
  }

  ~Visual()
  {
    manager_->destroySceneNode(node_);
  }

  Ogre::SceneManager *const manager_ = 0;
  Ogre::SceneNode *const node_ = 0;
  rviz::Arrow head_;
  rviz::Arrow tail_;
};

/**
  Le code pour dessiner un FourWheelSteering est rassemblé dans cette classe.
  Les classes suivantes l'utilisent en adaptant chaque type de message.
*/
struct DisplayHelper
{
  DisplayHelper(rviz::Display *display)
  : color_("Color", Qt::white, {}, display)
  , scale_("Size", .1, {}, display)
  {
  }

  void do_reset()
  {
    visual_.reset();
  }

  void do_processMessage(const four_wheel_steering_msgs::FourWheelSteeringStamped &m, rviz::DisplayContext *context_, Ogre::SceneNode *scene_node_)
  {
    Ogre::Vector3 p;
    Ogre::Quaternion q;
    if(!context_->getFrameManager()->getTransform(m.header.frame_id, m.header.stamp, p, q))
    {
      return;
    }
    if(!visual_)
    {
      visual_.reset(new Visual(context_->getSceneManager(), scene_node_));
    }
    visual_->node_->setPosition(p);
    visual_->node_->setOrientation(q);
    auto c = color_.getOgreColor();
    auto s = scale_.getFloat();
    visual_->head_.setColor(c);
    visual_->head_.setOrientation(
      Ogre::Quaternion(Ogre::Radian(m.data.front_steering_angle), Ogre::Vector3::UNIT_Z) *
      Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));
    visual_->head_.set(m.data.speed, s, 2 * s, 2 * s);
    visual_->tail_.setShaftColor(c);
    visual_->tail_.setOrientation(
      Ogre::Quaternion(Ogre::Radian(m.data.rear_steering_angle), Ogre::Vector3::UNIT_Z) *
      Ogre::Quaternion(Ogre::Degree(-90), Ogre::Vector3::UNIT_Y));
    visual_->tail_.set(-m.data.speed, s, 0, 0);
  }

  std::unique_ptr<Visual> visual_;
  rviz::ColorProperty color_;
  rviz::FloatProperty scale_;
};

struct rviz_display_FourWheelSteering
: TopicDisplay
, DisplayHelper
{
  rviz_display_FourWheelSteering()
  : DisplayHelper(this)
  {
  }

  Q_OBJECT

  void subscribe() override
  {
    do_subscribe(&rviz_display_FourWheelSteering::processMessage, this);
  }

  void reset() override
  {
    do_reset();
    TopicDisplay::reset();
  }

  void processMessage(const four_wheel_steering_msgs::FourWheelSteering::ConstPtr &in)
  {
    four_wheel_steering_msgs::FourWheelSteeringStamped out;
    set_header(out.header);
    out.data = *in;
    do_processMessage(out, context_, scene_node_);
  }
};

struct rviz_display_FourWheelSteeringStamped
: rviz::MessageFilterDisplay<four_wheel_steering_msgs::FourWheelSteeringStamped>
, DisplayHelper
{
  rviz_display_FourWheelSteeringStamped()
  : DisplayHelper(this)
  {
  }

  Q_OBJECT

  void reset() override
  {
    do_reset();
    MFDClass::reset();
  }

  void processMessage(const four_wheel_steering_msgs::FourWheelSteeringStamped::ConstPtr &m) override
  {
    do_processMessage(*m, context_, scene_node_);
  }
};

PLUGINLIB_EXPORT_CLASS(v::graphic::rviz_display_FourWheelSteering, rviz::Display);
PLUGINLIB_EXPORT_CLASS(v::graphic::rviz_display_FourWheelSteeringStamped, rviz::Display);

}
}
}

#endif
