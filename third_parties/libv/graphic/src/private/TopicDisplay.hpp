#include <libv/graphic/found/rviz>
#if \
  defined LIBV_GRAPHIC_RVIZ_FOUND &&\
  1
#include <rviz/message_filter_display.h>
#include <rviz/properties/tf_frame_property.h>

namespace v {
namespace graphic {

struct TopicDisplay: rviz::Display
{
  Q_OBJECT
  Q_SLOT void onEnable() override;
  void onDisable() override;
  void onInitialize() override;
  void setTopic(const QString &, const QString &) override;
  virtual void subscribe() = 0;
  ros::Subscriber subscriber_;
  rviz::RosTopicProperty topic_;
  rviz::TfFrameProperty frame_;
protected:
  TopicDisplay();
  void reset() override;
  void set_header(std_msgs::Header &) const;

  /**
    Les classes dérivées utiliseront cette fonction dans l'implémentation de la fonction subscribe() pour s'abonner au topic.
  */
  template<class... X>
  void do_subscribe(X &&...x)
  {
    subscriber_ = update_nh_.subscribe(topic_.getTopicStd(), 1, x...);
  }
};

}
}

#endif
