/**

\file
Définition de la classe v::graphic::TopicDisplay, dont des plugins rviz pourront dériver.
Les déclarations sont dans TopicDisplay.hpp.

\author Alexis Wilhelm (2018)

*/

/**

\class v::graphic::TopicDisplay
Une classe à dériver pour faciliter l'écriture de plugins pour rviz.

rviz propose une classe rviz::MessageFilterDisplay pour faciliter l'écriture des plugins qui affichent des messages de type xxxStamped, qui ont un champ `header`, mais il ne propose rien pour afficher des messages qui n'ont pas de champ `header`.
Cette classe essaie de combler cette lacune.

L'idée est de transformer les messages xxx en messages xxxStamped en ajoutant un champ `header`, puis de les afficher comme si c'étaient des vrais messages xxxStamped.

*/

#include "TopicDisplay.hpp"
#if \
  defined LIBV_GRAPHIC_RVIZ_FOUND &&\
  1

namespace v {
namespace graphic {

/**
  Commence à initialiser cet affichage.
  On fait tout ce qu'on peut, en sachant qu'on n'a pas encore accès à `context_`.
*/
TopicDisplay::TopicDisplay()
: topic_("Topic", {}, "geometry_msgs/Twist", {}, this, SLOT(onEnable()))
, frame_("Reference Frame", "base_link", {}, this)
{
}

/**
  Termine d'initialiser cet affichage.
  On a maintenant accès à tous les membres de rviz::Display.
*/
void TopicDisplay::onInitialize()
{
  frame_.setFrameManager(context_->getFrameManager()); // Ça doit être fait ici parce que `context_` n'est pas initialisé dans le constructeur.
}

/**
  Cette fonction est appelée par rviz quand cet affichage est initialisé avec un topic (bouton «Add», onglet «By topic»).
*/
void TopicDisplay::setTopic(const QString &topic, const QString &)
{
  topic_.setString(topic);
}

/**
  Cette fonction est appelée par rviz quand cette visualisation est activée.
  On l'appelle aussi quand l'utilisateur choisit un topic.
*/
void TopicDisplay::onEnable()
{
  reset();
  if(isEnabled())
  {
    subscribe();
  }
}

/**
  Cette fonction est appelée par rviz quand cette visualisation est désactivée.
*/
void TopicDisplay::onDisable()
{
  onEnable();
}

/**
  Les classes dérivées redéfiniront cette fonction pour effacer cet affichage.
*/
void TopicDisplay::reset()
{
  subscriber_.shutdown();
  Display::reset();
}

/**
  Les classes dérivées utiliseront cette fonction dans l'implémentation de la fonction processMessage() pour initialiser le champ `header` de leur message.
*/
void TopicDisplay::set_header(std_msgs::Header &m) const
{
  m.frame_id = frame_.getFrameStd();
}

}
}

#endif
