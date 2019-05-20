:orphan:

.. _rosvita_stop-label:

**************
rosvita_stop
**************

.. code-block:: bash

   #!/bin/bash

   echo "Check if ROSVITA is already running..."
   if [[ $(docker ps -a | grep rosvita | wc -l) > 0 ]]; then
      echo "YES"
      echo "Stopping ROSVITA"
      docker stop rosvita
   else
      echo "NO"
      exit 0
   fi

   if [[ $(docker ps -a | grep rosvita | wc -l) > 0 ]]; then
      echo "Failed"
      exit 1
   else
      echo "OK"
      echo "ROSVITA has been stopped successfully."
   fi

