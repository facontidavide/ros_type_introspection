^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_type_introspection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
