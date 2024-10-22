search_dir=ros2_bags
for entry in "$search_dir"/*
do
  echo -n "-i $entry "
done