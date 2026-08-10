function mu_save_ui(ax, finalPath, fmt)
% mu_save_ui — 原子写保存 UIAxes（exportgraphics），防止进程被杀留下损坏文件
%   ax        : UIAxes 句柄
%   finalPath : 最终路径（含扩展名）
%   fmt       : 'png' | 'eps'
% 实现：先写 "<name>.tmp<ext>"，exportgraphics 成功后再 movefile 原子改名。
[dr, nm, ext] = fileparts(finalPath);
if isempty(dr), dr = '.'; end
tmpPath = fullfile(dr, [nm '.tmp' ext]);
if exist(tmpPath,'file'), delete(tmpPath); end
try
    if strcmpi(fmt,'eps')
        exportgraphics(ax, tmpPath, 'ContentType','vector', 'BackgroundColor','white');
    else
        exportgraphics(ax, tmpPath, 'Resolution',300, 'BackgroundColor','white');
    end
    movefile(tmpPath, finalPath, 'f');
catch ME
    if exist(tmpPath,'file'), delete(tmpPath); end
    rethrow(ME);
end
end
