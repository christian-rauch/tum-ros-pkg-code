
ACTIONS=action/ArmHand.action action/Ptu.action action/Strings.action

.PHONY: cogman_msgs_all
cogman_msgs_all: .genaction all

include $(shell rospack find mk)/cmake.mk

.genaction: $(ACTIONS)
	for a in $(ACTIONS); do \
    rosrun actionlib genaction.py `rospack find cogman_msgs` `basename $$a` ;\
  done
	touch .genaction
