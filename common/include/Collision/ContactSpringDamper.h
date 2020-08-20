/*!
 * @file ContactSpringDamper.h
 * @brief Spring Damper based Contact dynamics
 */

#ifndef CONTACT_SPRING_DAMPER_H
#define CONTACT_SPRING_DAMPER_H

#include "ContactConstraint.h"
#include "Dynamics/FloatingBaseModel.h"

/*!
 * Class representing contact dynamics for a FloatingBaseModel
 * This computes external forces to apply bodies which are in contact with the ground
 */
template <typename T>
class ContactSpringDamper : public ContactConstraint<T> {
 public:

  /*!
   * Construct a new contact dynamics model
   * @param model : FloatingBaseModel for the contact dynamics
   */
  ContactSpringDamper(FloatingBaseModel<T>* model)
      : ContactConstraint<T>(model) {
    _nGC = CC::_model->_nGroundContact;
    for (size_t i(0); i < _nGC; ++i) {
      _tangentialDeflections.push_back(Vec2<T>::Zero());
    }
    deflectionRate.resize(_nGC);
  }

  virtual ~ContactSpringDamper() {}

  /*!
   * Run the ground contact model for a single collision plane on a list of
   * ground contact points. The ground is allowed to deform in the tangential
   * direction, but not the normal direction. Instead, the foot penetrates the ground
   * The ground also "remembers" its deformation between separate contact events.
   * (however it does spring back pretty quickly)
   * @param K Ground stiffness
   * @param D Ground damping
   * @param dt Timestep (used for deflection)
   */
  virtual void UpdateExternalForces(T K, T D, T dt);
  virtual void UpdateQdot(FBModelState<T>& state) {
    (void)state; /* Do nothing */
  }

 protected:
  size_t _nGC;

  void _groundContactWithOffset(T K, T D);

  vectorAligned<Vec2<T>> deflectionRate;
  vectorAligned<Vec2<T>> _tangentialDeflections;
};

#endif
