function mu_save_result(result, outDir, varargin)
% mu_save_result — 将一次完整的多无人机规划结果精致地落盘
%
% 用法:
%   mu_save_result(result, outDir)                      单用例 -> muav_<tag>_<ts>.mat
%   mu_save_result(result, outDir, 'tag', 'my')         自定义文件名 tag
%   mu_save_result(result, outDir, 'experiment', 'exp1') 追加进实验集合 exp1.mat（跨用例对比）
%   mu_save_result(result, outDir, 'index', true)       同时刷新 results/index.json 可读清单（默认开）
%
% 输入 result 建议字段:
%   .tag/.mode/.difficulty/.scene/.trajs/.bestX/.bestCost/.curve/.opt/.metrics
%   （缺字段会被安全地忽略，并记入 meta）
%
% 精致设计要点:
%   1. 自描述 meta: schemaVersion/savedAtUTC/matlabVersion/fields/missingFields/generator
%   2. 完整可复现: 同时存 bestX + scene，可用 mu_decode 重新生成 trajs（trajs 仅作快查副本）
%   3. 零残留: save 直接写最终文件名，不依赖临时文件改名
%   4. 轻量: terrainF 等函数句柄不可序列化 -> 存 terrainFInfo，load 端提示重建
%   5. 可读索引: 每次保存后追加一条记录到 results/index.json，无需 load 即可浏览全部结果
%   6. 实验集合: 同一 experiment 名的结果追加进单个 .mat 的 runs 字段，便于批量对比
%   7. 保存前校验: 检查 trajs 与 scene.nUAV 维度一致性，避免存坏数据

p = inputParser;
addParameter(p,'tag', '', @ischar);
addParameter(p,'experiment', '', @ischar);
addParameter(p,'index', true, @islogical);
parse(p, varargin{:});
tag     = p.Results.tag;
expName = p.Results.experiment;
doIndex = p.Results.index;

if nargin < 2 || isempty(outDir)
    outDir = fullfile(fileparts(mfilename('fullpath')), 'results');
end
if ~exist(outDir,'dir'), mkdir(outDir); end

% ---- 规范化 tag ----
if isempty(tag)
    if isfield(result,'tag') && ~isempty(result.tag)
        tag = result.tag;
    else
        m = 'mode'; dflt = 'scene'; nU = '';
        if isfield(result,'mode'), m = result.mode; end
        if isfield(result,'difficulty'), dflt = result.difficulty; end
        if isfield(result,'scene') && isfield(result.scene,'nUAV')
            nU = sprintf('_n%d', result.scene.nUAV);
        end
        tag = sprintf('%s_%s%s', m, dflt, nU);
    end
end
tag = matlab.lang.makeValidName(tag);

% ---- 保存前校验：trajs 与 scene.nUAV 维度一致 ----
validateResult(result);

% ---- 组装自描述 meta ----
allFields = {'tag','mode','difficulty','scene','trajs','bestX','bestCost','curve','opt','metrics'};
present   = isfield(result, allFields);
meta = struct();
meta.schemaVersion = '2.0';
meta.savedAtUTC    = char(datetime('now','TimeZone','UTC'));
meta.savedAtLocal  = char(datetime('now'));
meta.matlabVersion = version();
meta.fields        = allFields(present);
meta.missingFields = allFields(~present);
meta.generator     = 'mu_save_result';

% ---- 处理不可序列化的函数句柄字段 ----
stored = result;
if isfield(stored,'scene') && isfield(stored.scene,'terrainF') && isa(stored.scene.terrainF,'function_handle')
    fhSrc = functions(stored.scene.terrainF);
    stored.scene.terrainFInfo = struct('function', fhSrc.function, 'file', fhSrc.file);
    stored.scene = rmfield(stored.scene, 'terrainF');
end

% ---- 落盘 ----
if isempty(expName)
    % 单用例：独立 .mat
    ts = char(datetime('now','Format','yyyy-MM-dd_HHmmss'));
    fname = fullfile(outDir, sprintf('muav_%s_%s.mat', tag, ts));
    save(fname, 'stored', 'meta', '-v7.3');
    fprintf('  saved result: %s\n', fname);
    if doIndex
        appendIndex(outDir, struct('file', fname, 'tag', tag, 'meta', meta, ...
            'bestCost', getOrEmpty(result,'bestCost'), ...
            'mode', getOrEmpty(result,'mode'), ...
            'difficulty', getOrEmpty(result,'difficulty')));
    end
else
    % 实验集合：追加进单个 .mat 的 runs 字段
    en = matlab.lang.makeValidName(expName);
    fname = fullfile(outDir, sprintf('exp_%s.mat', en));
    if exist(fname,'file')
        d = load(fname, 'runs', 'expMeta');
        runs = d.runs; expMeta = d.expMeta;
    else
        runs = {};
        expMeta = struct('name', en, 'schemaVersion', '2.0', ...
            'createdUTC', char(datetime('now','TimeZone','UTC')));
    end
    runs{end+1} = struct('tag', tag, 'result', stored, 'meta', meta);
    save(fname, 'runs', 'expMeta', '-v7.3');
    fprintf('  appended to experiment set: %s (%d runs)\n', fname, numel(runs));
end
end

% ============ 子函数 ============
function validateResult(result)
% 软校验：trajs 条数应与 scene.nUAV 一致；不一致仅警告，不阻断
if isfield(result,'scene') && isfield(result.scene,'nUAV') && isfield(result,'trajs')
    if numel(result.trajs) ~= result.scene.nUAV
        warning('mu_save_result: trajs count (%d) != scene.nUAV (%d); saving anyway.', ...
            numel(result.trajs), result.scene.nUAV);
    end
end
end

function v = getOrEmpty(s, f)
if isfield(s,f) && ~isempty(s.(f))
    v = s.(f);
else
    v = '';
end
end

function appendIndex(outDir, rec)
% 把一条记录追加进 results/index.json（可读清单，便于不 load 即可浏览）
idxFile = fullfile(outDir, 'index.json');
if exist(idxFile,'file')
    try
        txt = fileread(idxFile);
        list = jsondecode(txt);
    catch
        list = struct('entries', {});
    end
else
    list = struct('entries', {});
end
if ~isfield(list,'entries'), list.entries = {}; end
if isstruct(list) && isscalar(list) && isfield(list,'entries') && iscell(list.entries)
    list.entries{end+1} = rec;
else
    list = struct('entries', {rec});
end
fid = fopen(idxFile, 'w');
if fid > 0
    fprintf(fid, '%s', jsonencode(list, 'PrettyPrint', true));
    fclose(fid);
end
end
