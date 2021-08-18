function rtn = cp_vicon_process_subject(trialName)

  % First parse subject specific data
  C = strsplit(trialName,'_');
  fileName = ['./',C{1},'_',C{2},'_',C{3}];

  if exist(fileName,'file') ~= 2
    fprintf('\n\tSubject file not found.\n');
    rtn = [];
    return
  end

  fid = fopen(fileName);

  C = textscan(fid,'%s %s');

  for i=1:numel(C{1})
    if i == 1 || i == 2 || i==3
      rtn.(C{1}{i}) = C{2}{i};
    else
      rtn.(C{1}{i}) = str2num(C{2}{i});
    end
  end
  fclose(fid);
end
