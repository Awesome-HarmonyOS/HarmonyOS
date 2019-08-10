## This is a boilerplate file for Google opensource projects.
## To make it useful, replace <<TEXT>> with actual text for your project.
## Also, look at comments with "## double hashes" to see if any are worth
## uncommenting or modifying.

%define	RELEASE	1
%define rel     %{?CUSTOM_RELEASE} %{!?CUSTOM_RELEASE:%RELEASE}
%define	prefix	/usr

Name: %NAME
Summary: Lightweight C unit testing framework.
Version: %VERSION
Release: %rel
Group: Development/Libraries
URL: http://code.google.com/p/cmockery
License: Apache
Vendor: Google
Packager: Google Inc. <opensource@google.com>
Source: http://%{NAME}.googlecode.com/files/%{NAME}-%{VERSION}.tar.gz
Distribution: Redhat 7 and above.
Buildroot: %{_tmppath}/%{name}-root
Prefix: %prefix

%description
The %name package contains a lightweight library to simplify and generalize the
process of writing unit tests for C applications.

%package devel
Summary: Lightweight C unit testing framework.
Group: Development/Libraries
Requires: %{NAME} = %{VERSION}

%description devel
The %name package contains static and debug libraries and header files for the
development of test applications using %name.

%changelog
    * Mon Aug 25 2008 <opensource@google.com>
    - First draft

%prep
%setup

%build
./configure
make prefix=%prefix

%install
rm -rf $RPM_BUILD_ROOT
make prefix=$RPM_BUILD_ROOT%{prefix} install

%clean
rm -rf $RPM_BUILD_ROOT

%files
%defattr(-,root,root)

## Mark all installed files within /usr/share/doc/{package name} as
## documentation.  This depends on the following two lines appearing in
## Makefile.am:
##     docdir = $(prefix)/share/doc/$(PACKAGE)-$(VERSION)
##     dist_doc_DATA = AUTHORS COPYING ChangeLog INSTALL NEWS README
%docdir %{prefix}/share/doc/%{NAME}-%{VERSION}
%{prefix}/share/doc/%{NAME}-%{VERSION}/*

%{prefix}/lib/libcmockery.so.0
%{prefix}/lib/libcmockery.so.0.0.0


%files devel
%defattr(-,root,root)

%{prefix}/include/google
%{prefix}/lib/libcmockery.a
%{prefix}/lib/libcmockery.la
%{prefix}/lib/libcmockery.so
