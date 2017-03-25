^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ros_type_introspection
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
