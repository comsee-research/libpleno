/**

\file
Des plugins rviz pour afficher des messages de type geometry_msgs::Twist ou geometry_msgs::TwistStamped.

Les plugins sont exportés avec PLUGINLIB_EXPORT_CLASS dans ce fichier, et avec le fichier `rviz.xml` situé à la racine de ce package.

\author Alexis Wilhelm (2018)

On veut dessiner une flèche correspondant à l'axe X dans t secondes.
Le détail des calculs pour trouver les expressions dans DisplayHelper::do_processMessage est dans le fichier "twist.mac" dans ce dossier.

*/

#include "TopicDisplay.hpp"
#if \
  defined LIBV_GRAPHIC_RVIZ_FOUND &&\
  1
#include <nav_msgs/Odometry.h>
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
  Le code pour dessiner un Twist est rassemblé dans cette classe.
  Les classes suivantes l'utilisent en adaptant chaque type de message.
*/
struct DisplayHelper
{
  DisplayHelper(rviz::Display *display)
  : color_("Color", Qt::white, {}, display)
  , scale_("Size", .1, {}, display)
  , duration_("Duration", 1, {}, display)
  {
  }

  void do_reset()
  {
    visual_.reset();
  }

  void do_processMessage(const geometry_msgs::TwistStamped &m, rviz::DisplayContext *context_, Ogre::SceneNode *scene_node_)
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
    const auto h = color_.getOgreColor();
    visual_->head_.setColor(h);
    visual_->tail_.setShaftColor(h);
    const auto s = scale_.getFloat();
    const auto t = duration_.getFloat();
    const auto v = rviz::vector3MsgToOgre(m.twist.linear);
    const auto w = rviz::vector3MsgToOgre(m.twist.angular);
    const Ogre::Real A = w.length(); // vitesse angulaire
    const Ogre::Real x = v.x, y = v.y, z = v.z; // vitesse linéaire
    auto position = v * t;
    auto direction = Ogre::Vector3::UNIT_X;
    if(A > 1e-3)
    {
      const Ogre::Real a = w.x / A, b = w.y / A, c = w.z / A; // axe de rotation
      const Ogre::Real C = std::cos(A * t), S = std::sin(A * t);
      position = {
        (a * (a * x + b * y + c * z) * (A * t - S) + (1 - C) * (b * z - c * y) + x * S) / A,
        (b * (a * x + b * y + c * z) * (A * t - S) + (1 - C) * (c * x - a * z) + y * S) / A,
        (c * (a * x + b * y + c * z) * (A * t - S) + (1 - C) * (a * y - b * x) + z * S) / A};
      direction = {a * a * (1 - C) + C, a * b * (1 - C) + c * S, a * c * (1 - C) - b * S};
    }
    visual_->head_.setPosition(position);
    visual_->head_.setDirection(direction);
    visual_->head_.set(5 * s, s, 2 * s, 2 * s);
    visual_->tail_.setDirection(position);
    visual_->tail_.set(position.length(), s / 2, 0, 0);
  }

  std::unique_ptr<Visual> visual_;
  rviz::ColorProperty color_;
  rviz::FloatProperty scale_;
  rviz::FloatProperty duration_;
};

struct rviz_display_Twist
: TopicDisplay
, DisplayHelper
{
  rviz_display_Twist()
  : DisplayHelper(this)
  {
  }

  Q_OBJECT

  void subscribe() override
  {
    do_subscribe(&rviz_display_Twist::processMessage, this);
  }

  void reset() override
  {
    do_reset();
    TopicDisplay::reset();
  }

  void processMessage(const geometry_msgs::Twist::ConstPtr &in)
  {
    geometry_msgs::TwistStamped out;
    set_header(out.header);
    out.twist = *in;
    do_processMessage(out, context_, scene_node_);
  }
};

struct rviz_display_TwistStamped
: rviz::MessageFilterDisplay<geometry_msgs::TwistStamped>
, DisplayHelper
{
  rviz_display_TwistStamped()
  : DisplayHelper(this)
  {
  }

  Q_OBJECT

  void reset() override
  {
    do_reset();
    MFDClass::reset();
  }

  void processMessage(const geometry_msgs::TwistStamped::ConstPtr &m) override
  {
    do_processMessage(*m, context_, scene_node_);
  }
};

struct rviz_display_Odometry_twist
: rviz::MessageFilterDisplay<nav_msgs::Odometry>
, DisplayHelper
{
  rviz_display_Odometry_twist()
  : DisplayHelper(this)
  {
  }

  Q_OBJECT

  void reset() override
  {
    do_reset();
    MFDClass::reset();
  }

  void processMessage(const nav_msgs::Odometry::ConstPtr &in) override
  {
    geometry_msgs::TwistStamped out;
    out.header = in->header;
    out.header.frame_id = in->child_frame_id;
    out.twist = in->twist.twist;
    do_processMessage(out, context_, scene_node_);
  }
};

PLUGINLIB_EXPORT_CLASS(v::graphic::rviz_display_Twist, rviz::Display);
PLUGINLIB_EXPORT_CLASS(v::graphic::rviz_display_TwistStamped, rviz::Display);
PLUGINLIB_EXPORT_CLASS(v::graphic::rviz_display_Odometry_twist, rviz::Display);

}
}
}

#endif
