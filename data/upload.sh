#!/bin/bash



echo "compress index.htm"
gzip -k index.htm 


echo "Uploading compressed index.htm to ESP8266" 
curl -F "file=@index.htm.gz" logger.local/edit

rm index.htm.gz


echo "Done"



