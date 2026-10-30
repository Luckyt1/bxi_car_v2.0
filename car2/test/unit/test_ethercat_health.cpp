#include "motor/yiyou/cyclic_health.h"

#include <iostream>
#include <stdexcept>

using namespace std::chrono_literals;
using chassis::ethercat::CyclicHealth;
using chassis::ethercat::HealthDecision;

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

void transient_loss_does_not_publish_or_acknowledge()
{
  CyclicHealth health;
  auto last_feedback = 0ms;
  unsigned acknowledged = 7;
  const unsigned pending = 8;
  for (const auto now : {2ms, 4ms}) {
    require(health.observe_pdo(-1, 12) == HealthDecision::retry, "two lost frames may be retried");
    if (CyclicHealth::can_publish(-1, 12, now - last_feedback, false)) {
      last_feedback = now;
      acknowledged = pending;
    }
  }
  require(last_feedback == 0ms && acknowledged == 7, "invalid data must not refresh or acknowledge");
  require(health.observe_pdo(12, 12) == HealthDecision::accept, "good frame recovers transient loss");
  require(CyclicHealth::can_publish(12, 12, 6ms - last_feedback, false), "fresh complete frame publishes");
  require(health.pdo_failures() == 0, "success resets only the current consecutive burst");
  require(health.observe_pdo(0, 12) == HealthDecision::retry, "new loss burst starts from one");
}

void persistent_loss_and_deadline_still_fault()
{
  CyclicHealth health;
  require(health.observe_pdo(-1, 12) == HealthDecision::retry, "first miss");
  require(health.observe_pdo(9, 12) == HealthDecision::retry, "partial WKC is also invalid");
  require(health.observe_pdo(0, 12) == HealthDecision::fault, "third consecutive miss must fault");
  require(!CyclicHealth::can_publish(12, 12, 2ms, true), "recovery must not bypass latched fault");
  require(!CyclicHealth::feedback_expired(20ms), "existing strict deadline boundary retained");
  require(CyclicHealth::feedback_expired(20ms + 1us), "stale feedback must fault without three misses");
  require(!CyclicHealth::can_publish(12, 12, 21ms, false), "late good frame cannot erase elapsed deadline");
  require(!CyclicHealth::can_publish(9, 12, 1ms, false), "partial image cannot be published");
}

void al_retry_is_per_slave_and_does_not_wait_twenty_ms()
{
  CyclicHealth health;
  require(health.should_poll_al(10, 1), "normal AL polling cadence");
  require(!health.should_poll_al(11, 1), "healthy slave is not polled every cycle");
  require(health.observe_al(1, false, false) == HealthDecision::retry, "first timeout is retried");
  require(health.should_poll_al(11, 1), "failed slave is retried on the next cycle");
  require(!health.should_poll_al(11, 2), "retry does not add queries for healthy slaves");
  require(health.observe_pdo(12, 12) == HealthDecision::accept, "PDO stays independently healthy");
  require(health.al_failures(1) == 1, "good PDO does not erase AL failure count");
  require(health.observe_al(2, true, true) == HealthDecision::accept, "other slave remains healthy");
  require(health.observe_al(1, false, false) == HealthDecision::retry, "second timeout is retried");
  require(health.observe_al(1, false, false) == HealthDecision::fault, "third timeout faults same slave");
}

void al_recovery_and_explicit_device_fault_are_distinct()
{
  CyclicHealth health;
  health.observe_al(4, false, false);
  require(health.observe_al(4, true, true) == HealthDecision::accept, "OP response resets AL retry");
  require(!health.should_poll_al(13, 4), "recovered slave resumes normal poll cadence");
  require(health.observe_al(4, false, false) == HealthDecision::retry, "isolated new timeout is allowed");
  require(health.observe_al(4, true, false) == HealthDecision::fault, "explicit non-OP/error faults immediately");
  require(health.observe_al(1, true, false) == HealthDecision::fault, "no previous misses needed for device fault");
}
}  // namespace

int main()
{
  try {
    transient_loss_does_not_publish_or_acknowledge();
    persistent_loss_and_deadline_still_fault();
    al_retry_is_per_slave_and_does_not_wait_twenty_ms();
    al_recovery_and_explicit_device_fault_are_distinct();
    std::cout << "EtherCAT health policy tests passed\n";
    return 0;
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
