
(in-package :kipla-reasoning)

(def-fact-group object-classes
  (<- (obj-subtype ?a ?b)
    (obj-direct-subtype ?a ?b))
  (<- (obj-subtype ?a ?b)
    (obj-direct-subtype ?a ?tmp)
    (obj-subtype ?tmp ?b))
  (<- (obj-direct-subtype object cluster))
  (<- (obj-direct-subtype cluster icetea))
  (<- (obj-direct-subtype cluster mug))
  (<- (obj-direct-subtype cluster jug))
  (<- (obj-direct-subtype cluster placemat))
  (<- (obj-direct-subtype cluster coke)))
