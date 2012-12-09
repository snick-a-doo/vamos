#!/bin/bash
# Create a header dependency graph from the source files in a directory.
# Output is in the DOT language.  The 'dot' application is part of
# graphviz <http://www.graphviz.org>
#
# Usage: graph.sh DIRECTORY [VIEWER]
#
# Serving suggestion:
# graph.sh world | dot -Tps -o /tmp/graph.ps && evince /tmp/graph.ps

if [ "$#" == "2" ]; then
    $0 $1 | dot -Tps -o /tmp/graph.ps && $2 /tmp/graph.ps
    exit 1
fi

if [ "$#" != "1" ]; then
    echo "Usage: graph.sh DIRECTORY [VIEWER]"
    exit 1
fi

cd $1

echo "digraph $(basename $PWD) {"

# Find and format the header dependencies
grep "#include.*\.h\"" *.cc *.h |\
sed 's!\(.*\)\.[cch]\+:#include "\(.*\)\.[cch]\+"!"\1" -> "\2";!' |\
grep -v "^\(.*\) -> \1" |\
grep -v "test_"

# Make sure independent headers are shown.
ls -1 *.h | sed 's/\.h$//'

echo "}"
