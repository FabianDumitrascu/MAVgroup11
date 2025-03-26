#!/bin/bash

values=$(grabc | sed -n '2p')
echo "values = $values"
