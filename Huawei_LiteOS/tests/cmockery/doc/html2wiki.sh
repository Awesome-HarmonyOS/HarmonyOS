#!/bin/bash
#
# Translate really simple html to googlecode.com wiki.
#
# Usage: cat input.html | html2wiki.sh > outputwiki.txt
#
# Most of this script is simple sed substitutions with an awk script to handle
# hierarchical lists.

# Awk program to escape all instances of * outside of <listing></listing>
awk '
BEGIN { in_listing = 0; }
/<[Ll][Ii][Ss][Tt][Ii][Nn][Gg]>/ { in_listing = 1; }
/<\/[Ll][Ii][Ss][Tt][Ii][Nn][Gg]>/ { in_listing = 0; }
/.*/ {
  if (in_listing) {
    print $0;
  } else {
    print gensub("*", "`*`", "g", $0)
  }
}' | \
# Awk program to convert hierachical unordered and ordered lists into
# googlecode wiki list markup.  This is limited to converting very simple
# html lists in the form:
#
# <ul>
#   <li>item 1</li>
#   ...
#   <li>item N</li>
# </ul>
#
# This script also removes leading spaces from all lines outside of <listing>
# sections.
awk '
BEGIN {
  list_type_none = 0;
  list_type_ordered = 1;
  list_type_unordered = 2;
  # Number of nested lists.
  list_depth = 0;
  # Number of items in the list.
  list_items[list_depth] = 0;
  # Type of list.
  list_type[list_depth] = list_type_none;
  # Do nott strip whitespace from listing sections.
  in_listing = 0;
}

# Generate a string of indent spaces.
function list_indent(indent) {
  format = sprintf("%%%ds", indent);
  return sprintf(format, "");
}

/<[Ll][Ii][Ss][Tt][Ii][Nn][Gg]>/ { in_listing = 1; }
/<\/[Ll][Ii][Ss][Tt][Ii][Nn][Gg]>/ { in_listing = 0; }

# Process all lines non-blank lines.
/^.*$/ {
  # Remove leading white space.
  if (!in_listing) {
    output_string = gensub(/^ */, "", 1, $0);
  } else {
    output_string = $0;
  }
  search_string = output_string

  # Replace list tags with googlecode wiki markup.
  while (match(search_string, /<[^>]*>/, matches)) {
    tag = matches[0];
    search_string = substr(search_string,
                           matches[0, "start"] + matches[0, "length"]);
    if (match(tag, /^<[Uu][Ll]>$/)) {
      list_depth++;
      list_type[list_depth] = list_type_unordered;
      list_items[list_depth] = 0;
      output_string = gensub(tag, "", 1, output_string);
    } else if (match(tag, /^[Oo][Ll]>$/)) {
      list_depth++;
      list_type[list_depth] = list_type_ordered;
      list_items[list_depth] = 0;
      output_string = gensub(tag, "", 1, output_string);
    } else if (match(tag, /^<\/[Ll][Ii]>$/)) {
      output_string = gensub(tag, "", 1, output_string);
    } else if (list_depth) {
      if (match(tag, /^<[Ll][Ii]>$/)) {
        if (list_type[list_depth] == list_type_unordered) {
          output_string = gensub(tag, list_indent(list_depth) "* ", 1,
                                 output_string);
        } else if (list_type[list_depth] == list_type_ordered) {
          output_string = gensub(tag, list_indent(list_depth) "# ", 1,
                                 output_string);
        }
      } else if (match(tag, /^<\/[Uu][Ll]>$/) ||
                 match(tag, /^<\/[Ou][Ll]>$/)) {
        output_string = gensub(tag, "", 1, output_string);
        list_depth --;
      }
    }
  }
  # If a list is being parsed then filter blank lines.
  if (list_depth == 0 || length(output_string)) {
    print output_string 
  }
}
' | \
# This sed program translates really simple html into wiki suitable for
# googlecode.com.
#
# Supported tags:
# <p>
# <br>
# <h1>
# <h2>
# <h3>
# <h4>
# <h5>
# <b>
# <i>
# <a href="#.*">.*</a>
# <a href=".*">.*</a>
# <a name=".*'>.*</a>
#
# Supported entities:
# &gt;
# &lt;
#
# Limitations:
# * Anchors must be on a single line and must contain one of either the name or
#   href attributes.
# * Href of local anchors (href="#.*") should be set to the name of a heading
#   within the document.  If the heading contains spaces the href should
#   contain underscores.
# * All external links are relative to
#   http://cmockery.googlecode.com/svn/trunk/doc/
sed -r '
s@<[Pp]>@\n@g;
s@<[[Bb][Rr]]>@\n@g;
s@</?[Hh]1>@=@g;
s@</?[Hh]2>@==@g;
s@</?[Hh]3>@===@g;
s@</?[Hh]4>@====@g;
s@</?[Hh]5>@====@g;
s@</?[Bb]>@*@g;
s@</?[Ii]>@_@g;
s@<[Ll][Ii][Ss][Tt][Ii][Nn][Gg]>@{{{@g;
s@</[Ll][Ii][Ss][Tt][Ii][Nn][Gg]>@}}}@g;
s@<[Aa].*?href="#(.*)?">(.*)?</[Aa]>@[#\1 \2]@g;
s@<[Aa].*?href="(.*)?">(.*)?</[Aa]>@[http://cmockery.googlecode.com/svn/trunk/doc/\1 \2]@g;
s@<[Aa].*?name="(.*)?">@@g;
s@</[Aa]>@@g;
s@<.*?>@@g;
s@&lt;@<@g;
s@&gt;@>@g;'
