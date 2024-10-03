clear
ROOT_DIR=fileparts(mfilename('fullpath'));

load("box_pickup_ihmc.mat")

csv_path = strcat(ROOT_DIR,"/csv/");
ret = exist(csv_path, "dir");
if !ret
  mkdir(csv_path)
endif

  
elements = whos();

num_elements = length(elements);

for i=1:num_elements
  name = elements(i).name;
  [int2str(i)," ",name];
  if (!strcmp(name,"csv_path")) && (!(strcmp(name,"ans")))
    if (!strcmp(name,'i')) && (!(strcmp(name,"num_elements")))
      if !strcmp(name,"elements")
        csvwrite([csv_path,name,".csv"], eval(name))  
      endif
    endif
  endif
endfor