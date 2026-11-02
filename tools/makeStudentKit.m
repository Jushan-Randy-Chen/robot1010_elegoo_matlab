function zipFile = makeStudentKit()
% MAKESTUDENTKIT  Build the self-contained student kit and its zip file.
%   zipFile = makeStudentKit()
% The kit is a flat folder: the template finds all of its files in the
% same folder. Output: dist/ElegooMappingKit/ and dist/ElegooMappingKit.zip
% (the dist folder is not in git).
%
% The zip file has no top folder. Windows "Extract All" and the Mac Finder
% both make one folder ElegooMappingKit (no folder in a folder).
%
% Before you build the kit, update the HTML guide after each change of
% STUDENT_GUIDE.md:   python tools/build_guide_html.py

root = fileparts(fileparts(mfilename("fullpath")));
basic = fullfile(root, "basic_wireless_control");
vslam = fullfile(root, "vslam_examples");
dist = fullfile(root, "dist");
kit = fullfile(dist, "ElegooMappingKit");

guideMd = dir(fullfile(vslam, "STUDENT_GUIDE.md"));
guideHtml = dir(fullfile(vslam, "STUDENT_GUIDE.html"));
if isempty(guideHtml) || guideHtml.datenum < guideMd.datenum
    warning("STUDENT_GUIDE.html is older than STUDENT_GUIDE.md. Run: python tools/build_guide_html.py");
end

if isfolder(kit)
    rmdir(kit, "s");
end
mkdir(kit);

copyfile(fullfile(basic, "ElegooRobotCar.m"), kit);
for name = ["elegooMappingTemplate.m", "ElegooSimCar.m", "calibrateElegooCamera.m", ...
        "STUDENT_GUIDE.md", "STUDENT_GUIDE.html"]
    copyfile(fullfile(vslam, name), kit);
end
copyfile(fullfile(vslam, "simAssets"), fullfile(kit, "simAssets"));
copyfile(fullfile(vslam, "docs"), fullfile(kit, "docs"));
copyfile(fullfile(vslam, "STUDENT_README.txt"), fullfile(kit, "README.txt"));

zipFile = fullfile(dist, "ElegooMappingKit.zip");
if isfile(zipFile)
    delete(zipFile);
end
items = dir(kit);
items = items(~ismember({items.name}, {'.', '..'}));
zip(zipFile, {items.name}, kit);
fprintf("Student kit: %s\n", zipFile);
end
