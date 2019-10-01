^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_type_introspection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.2 (2019-10-01)
------------------
* speed optimization (fix regression in 2.0)
* Contributors: Davide Faconti

2.0.1 (2019-10-01)
------------------
* fix (old abseil stuff)
* Contributors: Davide Faconti

2.0.0 (2019-10-01)
------------------
* removed abseil
* Merge branch 'master' of https://github.com/facontidavide/ros_type_introspection
* minor changes in the API
* Fix issue `#38 <https://github.com/facontidavide/ros_type_introspection/issues/38>`_
* Contributors: Davide Faconti

1.3.3 (2019-05-10)
------------------
* fix issue `#36 <https://github.com/facontidavide/ros_type_introspection/issues/36>`_
* Contributors: Davide Faconti

1.3.2 (2019-04-17)
------------------
* fix issue `#35 <https://github.com/facontidavide/ros_type_introspection/issues/35>`_
* Contributors: Davide Faconti

1.3.1 (2019-03-24)
------------------
* Merge pull request `#32 <https://github.com/facontidavide/ros_type_introspection/issues/32>`_ from aeudes/fix_large_array
* Fix invalid clamp and discard beaviour.
* Contributors: Alexandre Eudes, Davide Faconti

1.3.0 (2019-01-25)
------------------
* adding new policy
* fix compiler warnings
* added comment
* Contributors: Davide Faconti, Robert Haschke, YiweiHan, janEbert

1.2.0 (2018-11-12)
------------------
* fixed -Wreorder (`#24 <https://github.com/facontidavide/ros_type_introspection/issues/24>`_)
  * client code can now compile with -Werror
* solved compilation problem when convert_impl<float, float>
* Update README.md
* Contributors: Davide Faconti, moooeeeep

1.1.1 (2018-04-15)
------------------
* split the project into two packages to reduce dependencies
* Contributors: Davide Faconti

1.1.0 (2018-03-22)
------------------
* critical bug fix
* fixed compilation of test
* remove potential problem in other projects
* Contributors: Davide Faconti

1.0.2 (2018-02-08)
------------------
* fixing issue with blobs (detected when parsing sensor_msgs::Image)
* added test for sensor image
* Contributors: Davide Faconti

1.0.1 (2017-11-14)
------------------
* added return value to deserializeIntoFlatContainer
* bug fix
* fix compilation issue
* fix potential issue with static variables
* Contributors: Davide Faconti

1.0.0 (2017-10-12)
------------------
* Complete refactoring
* Contributors: Davide Faconti, Kartik Mohta, Ian Taylor, Mehdi Tlili 

0.8.0 (2017-08-30)
------------------
* fixing a serious issue with vectors which are too large
* Contributors: Davide Faconti

0.7.1 (2017-08-29)
------------------
* ros Time and Duration fixed
* tests fixed
* important API Change
* might fix issue reported in `#5 <https://github.com/facontidavide/ros_type_introspection/issues/5>`_. print_number made public
* adding inline to fix compilation error `#4 <https://github.com/facontidavide/ros_type_introspection/issues/4>`_
* maintain nanosecond precision for time/duration
* Contributors: Davide Faconti, Ian Taylor

0.6.3 (2017-06-26)
------------------
* speed up
* yet another bug fixed
* considerable speed improvement
* Contributors: Davide Faconti

0.6.2 (2017-06-23)
------------------
* bug fix. types where missing in conversion
* Contributors: Davide Faconti

0.6.1 (2017-06-22)
------------------
* fixed a bug in resize
* potential compilation problem fixed
* Contributors: Davide Faconti

0.6.0 (2017-06-20)
------------------
* moved the deserializing code
* new API
* fixing issue in resize (to be tested)
* fixed osx compilation failure due to implicit_instantiation of std::array
* Fix formating and typos
* Contributors: Bo Li, Davide Faconti, Sam Pfeiffer

0.5.1 (2017-04-02)
------------------
* fix the test
* fix tests?
* compilation fix
* typo fix
* test fixed
* Contributors: Davide Faconti

0.5.0 (2017-03-25)
------------------
* toStr changed
* Contributors: Davide Faconti

0.4.3 (2017-02-13)
------------------
* FIX: bug found in cache. reverting the recent change
* Contributors: davide

0.4.1 (2017-02-09)
------------------
* COSMETIC: more consistent code
* considerable speed improvement in applyNameTransform
* Contributors: Davide Faconti

0.4.0 (2017-02-06)
------------------
* critical bug fixed
* remove compilation warnings
* Update README.md
* Contributors: Davide Faconti

0.3.3 (2016-11-04)
------------------
* removed serious bug that might cause double free
* Contributors: davide

0.3.2 (2016-10-26)
------------------
* fixing tests (EXPECTED_EQ is more informative)
* changed the type of ROSType::baseName() and added SString::toStdString()

0.3.1 (2016-10-20)
------------------
* added BSD license
* added an alternative implementation of ShapeShifter

0.3.0 (2016-10-17)
-----------

* Doxygen added
* Moved to gtests instead of Catch.
* Final API (?)

0.2.0 (2016-10-17)
-----------

* All unit tests pass, but coverage is not very high.
* By default is uses the custom string implementation.
* Stable (?) API
