function zipFile = makeStudentKit()
% MAKESTUDENTKIT  Build the self-contained student kit and its zip file.
%   zipFile = makeStudentKit()
% The kit is a flat folder: the template finds all of its files in the
% same folder. Output: dist/ElegooMappingKit/ and dist/ElegooMappingKit.zip
% (the dist folder is not in git).

root = fileparts(fileparts(mfilename("fullpath")));
basic = fullfile(root, "basic_wireless_control");
vslam = fullfile(root, "vslam_examples");
dist = fullfile(root, "dist");
kit = fullfile(dist, "ElegooMappingKit");

if isfolder(kit)
    rmdir(kit, "s");
end
mkdir(kit);

copyfile(fullfile(basic, "ElegooRobotCar.m"), kit);
for name = ["elegooMappingTemplate.m", "ElegooSimCar.m", "calibrateElegooCamera.m"]
    copyfile(fullfile(vslam, name), kit);
end
copyfile(fullfile(vslam, "simAssets"), fullfile(kit, "simAssets"));
copyfile(fullfile(vslam, "STUDENT_README.txt"), fullfile(kit, "README.txt"));

zipFile = fullfile(dist, "ElegooMappingKit.zip");
if isfile(zipFile)
    delete(zipFile);
end
zip(zipFile, "ElegooMappingKit", dist);
fprintf("Student kit: %s\n", zipFile);
end
