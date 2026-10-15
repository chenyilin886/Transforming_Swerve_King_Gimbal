const fs = require('fs');
const path = require('path');
const source = "C:\\Users\\chen'yi'lin\\Desktop\\数据采集";
const names = ['yaw匀速中速.csv','yaw匀速高速.csv','yaw匀速低速.csv','yaw急停.csv','yaw变速.csv','yaw_sin波全频中幅值.csv','yaw_sin波全频高幅值.csv'];
const dt = 0.004;
const out = path.resolve('outputs/yaw_identification');
fs.mkdirSync(out,{recursive:true});
function solve(A,b) {
 A=A.map(r=>r.slice()); b=b.slice(); const n=b.length;
 for(let j=0;j<n;j++) {let k=j;for(let i=j+1;i<n;i++)if(Math.abs(A[i][j])>Math.abs(A[k][j]))k=i;
  [A[j],A[k]]=[A[k],A[j]];[b[j],b[k]]=[b[k],b[j]];
  if(Math.abs(A[j][j])<1e-12)return null;
  for(let i=j+1;i<n;i++){const q=A[i][j]/A[j][j];for(let k=j;k<n;k++)A[i][k]-=q*A[j][k];b[i]-=q*b[j];}
 }
 const p=Array(n).fill(0);for(let i=n-1;i>=0;i--){p[i]=(b[i]-A[i].reduce((s,v,k)=>s+(k>i?v*p[k]:0),0))/A[i][i];}return p;
}
function score(rows,p) {
 let ymean=rows.reduce((s,r)=>s+r.y,0)/rows.length, sse=0,sst=0;
 for(const r of rows){const yhat=r.x.reduce((s,v,j)=>s+v*p[j],0);sse+=(r.y-yhat)**2;sst+=(r.y-ymean)**2;}
 return {R2:1-sse/sst,RMSE:Math.sqrt(sse/rows.length),n:rows.length};
}
function fit(rows,cols=[0,1]) {
 const n=cols.length,A=Array.from({length:n},()=>Array(n).fill(0)), b=Array(n).fill(0);
 for(const r of rows)for(let j=0;j<n;j++){b[j]+=r.x[cols[j]]*r.y;for(let k=0;k<n;k++)A[j][k]+=r.x[cols[j]]*r.x[cols[k]];}
 const z=solve(A,b); if(!z)return null; const p=Array(4).fill(0);cols.forEach((c,i)=>p[c]=z[i]); return {p,...score(rows,p)};
}
function positiveFit(rows,cols) {
 let fits=[];for(const drop of [[],[0],[1],[0,1]]){const f=fit(rows,cols.filter(c=>!drop.includes(c)));if(f&&f.p[0]>=0&&f.p[1]>=0)fits.push(f);}
 return fits.sort((a,b)=>a.RMSE-b.RMSE)[0];
}
function makeRows(data,width) {
 const result=[];
 if(width===0){for(let i=1;i<data.length-1;i++)result.push({x:[(data[i+1][1]-data[i-1][1])/(2*dt),data[i][1],1,Math.sign(data[i][1])],y:data[i][2]});return result;}
 // Non-overlapping trapezoidal integral windows, divided by window duration.
 for(let i=0;i+width<data.length;i+=width){let w=0,t=0,sg=0;for(let j=0;j<=width;j++){const h=(j===0||j===width)?.5:1;w+=h*data[i+j][1];t+=h*data[i+j][2];sg+=h*Math.sign(data[i+j][1]);}
 result.push({x:[(data[i+width][1]-data[i][1])/(width*dt),w/width,1,sg/width],y:t/width});}return result;
}
let datasets=[],results=[];
for(const name of names){
 const lines=fs.readFileSync(path.join(source,name),'utf8').trim().split(/\r?\n/);
 const data=lines.slice(1).map(l=>l.split(',').map(Number));
 if(data.some(r=>r.length!==6||!r.every(Number.isFinite)))throw Error('Invalid numeric rows '+name);
 let intervals=[];for(let i=1;i<data.length;i++){const w=(data[i][1]+data[i-1][1])/2,d=data[i][0]-data[i-1][0];if(Math.abs(w)>1&&d/w>0&&d/w<.02)intervals.push(d/w);}
 intervals.sort((a,b)=>a-b);const dtMedian=intervals[Math.floor(intervals.length/2)];
 datasets.push({name,data,dtMedian});
 for(const width of [0,5,25,50]){const rows=makeRows(data,width),split=Math.floor(rows.length*.7),train=rows.slice(0,split),test=rows.slice(split);
  const models={};for(const [model,cols] of [['JB',[0,1]],['JB_offset',[0,1,2]],['JB_offset_friction',[0,1,2,3]]]){
   const f=fit(rows,cols),ft=fit(train,cols);models[model]=f&&ft?{...f,holdout:score(test,ft.p),trainP:ft.p}:null;
  }models.JB_positive=positiveFit(rows,[0,1]);results.push({name,width,models});}
}
let pooled=[];for(const d of datasets)pooled.push(...makeRows(d.data,0));
const global={JB:fit(pooled),JB_positive:positiveFit(pooled,[0,1]),JB_offset:fit(pooled,[0,1,2]),JB_offset_friction:fit(pooled,[0,1,2,3])};
// A virtual parameter pair is evaluated against unchanged measurements, and separately on synthetic data.
const virtualP=[0.014,0.09,0,0];
const stopData=datasets.find(d=>d.name==='yaw急停.csv').data;
const virtualMeasured={pooled:score(pooled,virtualP),stop:score(makeRows(stopData,0),virtualP),stop100ms:score(makeRows(stopData,25),virtualP)};
const synthetic=Array.from({length:10000},(_,i)=>{const t=i*dt,w=2*Math.sin(2*Math.PI*.5*t)+.8*Math.sin(2*Math.PI*1.7*t),a=2*2*Math.PI*.5*Math.cos(2*Math.PI*.5*t)+.8*2*Math.PI*1.7*Math.cos(2*Math.PI*1.7*t);return {x:[a,w,1,Math.sign(w)],y:virtualP[0]*a+virtualP[1]*w};});
let state=20261015;function random(){state=(1664525*state+1013904223)>>>0;return (state+.5)/4294967296;}
const ym=synthetic.reduce((s,r)=>s+r.y,0)/synthetic.length,ys=Math.sqrt(synthetic.reduce((s,r)=>s+(r.y-ym)**2,0)/synthetic.length);
const noisy=synthetic.map(r=>({...r,y:r.y+.01*ys*Math.sqrt(-2*Math.log(random()))*Math.cos(2*Math.PI*random())}));
const syntheticResult={noiseless:score(synthetic,virtualP),noisy:score(noisy,virtualP),refit:fit(noisy)};
fs.writeFileSync(path.join(out,'results.json'),JSON.stringify({dt,columns:'first three columns: yaw angle rad, velocity rad/s, torque Nm; high-amplitude I0-I2 assumed same mapping',sources:datasets.map(d=>({name:d.name,n:d.data.length,dtMedian:d.dtMedian})),global,results,virtualP,virtualMeasured,syntheticResult},null,2));
fs.writeFileSync(path.join(out,'synthetic_virtual.csv'),'time_s,omega_rad_s,alpha_rad_s2,torque_ideal_Nm,torque_noisy_Nm\n'+synthetic.map((r,i)=>[i*dt,r.x[1],r.x[0],r.y,noisy[i].y].join(',')).join('\n'));
console.log(JSON.stringify({sources:datasets.map(d=>({name:d.name,n:d.data.length,dtMedian:d.dtMedian})),global,raw:results.filter(r=>r.width===0),integral100ms:results.filter(r=>r.width===25),virtualMeasured,syntheticResult},null,2));
const f=(x,n=6)=>x.toFixed(n);
const stop=results.find(r=>r.width===25&&r.name==='yaw急停.csv').models.JB;
const gp=fit(datasets.flatMap(d=>makeRows(d.data,25)));
const transfer=datasets.map(d=>({name:d.name,...score(makeRows(d.data,25),stop.p)}));
const resultPath=path.join(out,'results.json');
const saved=JSON.parse(fs.readFileSync(resultPath,'utf8'));
saved.global100ms=gp;saved.stopTransfer100ms=transfer;
fs.writeFileSync(resultPath,JSON.stringify(saved,null,2));
const report=`# Yaw 转动惯量 J 与阻尼 B 参数辨识

## 主要结果

- 综合用户指定的全部 7 份数据，采用 100 ms 积分窗口，推荐作为低频等效模型初值：J = ${f(gp.p[0])} kg·m²，B = ${f(gp.p[1])} N·m·s/rad，窗口平均扭矩拟合 R² = ${f(gp.R2)}，RMSE = ${f(gp.RMSE)} N·m。
- 以急停文件的全部有效数据、100 ms 积分窗口辨识两参数模型，J = ${f(stop.p[0])} kg·m²，B = ${f(stop.p[1])} N·m·s/rad。
- 对窗口平均扭矩的拟合 R² = ${f(stop.R2)}，RMSE = ${f(stop.RMSE)} N·m；按时间前 70% 训练、后 30% 验证，验证 R² = ${f(stop.holdout.R2)}。
- 急停参数是一组适合先用于急停工况、低频动态仿真的候选参数。它未能解释全部工况；上述两组参数均不能认定为已充分验证的系统真实 J、B。
- 如果要求把全部原始样本放在一起，使用同一组常数 J、B，最小二乘最优值为 J = ${f(global.JB.p[0],9)}，B = ${f(global.JB.p[1],9)}，R² 仅 ${f(global.JB.R2)}。在这个固定模型、采样周期和预处理下，任意更换常数 J、B 都不能超越该最小二乘的 R² 上限。

## 数据与假设

只分析用户指定的 7 份 CSV，共 55,000 行，没有纳入目录中的其他文件。来源目录：${source}。

${datasets.map(d=>`- ${d.name}：${d.data.length} 行；由角度与角速度估计的周期中位数 ${f(d.dtMedian*1000,4)} ms。`).join('\n')}

CSV 没有时间戳。统一假设每行间隔 Δt = 0.004 s，即 250 Hz。周期估计仅作为一致性检查，不能替代真实时间戳。若实际周期为 T，保持其他条件不变，J 应乘以 T/0.004，B 不变。未处理可能存在的通道延迟。

使用前三列分别作为 yaw 角度（rad）、角速度（rad/s）、扭矩（N·m）。高幅值正弦文件的表头仅为 I0、I1、I2，按其他文件同样的通道排列解释；该映射需要采集配置确认。假设扭矩已是 yaw 轴端实际扭矩，且与角速度同正方向；如果它是指令力矩或电机端力矩，结果只能理解为对应通道的等效参数。

## 模型与 R² 定义

主模型：τ = J·dω/dt + B·ω。J、B 均由普通最小二乘求解，不添加截距，不人为调节数据。

原始采样计算采用中心差分 α_i = (ω_(i+1) − ω_(i−1))/(2Δt)，每个文件仅舍去两端各一个样本。

积分计算采用互不重叠的 25 个采样间隔窗口（100 ms），使用梯形积分计算平均扭矩及平均速度，并令平均角加速度 = (ω_end − ω_start)/0.1。每条窗口有 26 个端点样本，邻窗只共享边界点；不完整尾窗舍弃。积分等式保持模型形式，可降低微分放大噪声的影响，也会移除高频误差，故其 R² 不能与原始样本 R² 混称。

R² = 1 − Σ(τ − τ_hat)²/Σ(τ − mean(τ))²，使用中心化定义，即使模型不含截距也不改用非中心化 R²。负 R² 表示模型误差大于使用扭矩均值的误差。

## 每个文件的两参数辨识

所有值均为各文件自身全量数据拟合的结果，未将负值强行改成正值。

| 文件 | 原始 J | 原始 B | 原始 R² | 100 ms J | 100 ms B | 100 ms R² |
|---|---:|---:|---:|---:|---:|---:|
${names.map(name=>{const a=results.find(r=>r.name===name&&r.width===0).models.JB,b=results.find(r=>r.name===name&&r.width===25).models.JB;return `| ${name} | ${f(a.p[0])} | ${f(a.p[1])} | ${f(a.R2,4)} | ${f(b.p[0])} | ${f(b.p[1])} | ${f(b.R2,4)} |`;}).join('\n')}

匀速工况的角加速度变化不足，J 容易受量化误差和未建模扰动影响；负 J 不具备被动机械惯量的物理意义。100 ms 各工况的 B 仍明显不同，说明两参数常数模型无法统一描述这些记录。

7 文件的 100 ms 窗口合并后最优拟合：J = ${f(gp.p[0])}，B = ${f(gp.p[1])}，R² = ${f(gp.R2)}。不能只报告急停文件的高 R²，而宣称全部数据均达到该水平。

## 急停参数的跨工况检验

以下将急停文件辨识出的 J、B 固定，直接用于其他文件的 100 ms 平均扭矩预测，不再重拟合。

| 文件 | R² | RMSE（N·m） |
|---|---:|---:|
${transfer.map(r=>`| ${r.name} | ${f(r.R2,4)} | ${f(r.RMSE,4)} |`).join('\n')}

按时间留出的验证没有跨窗口混用样本；但同一文件的留出验证并不等于独立重复实验验证。正弦文件的后段扭矩方差很小，会使留出 R² 为较大负值，完整结果与 RMSE 见 results.json。加偏置、加库仑摩擦的诊断模型也保留在该文件中，不能把其额外项的改善归功于仅有 J、B 的模型。

## 用户要求的虚拟 J、B

取 J_virtual = 0.014 kg·m²，B_virtual = 0.090 N·m·s/rad，与急停积分辨识结果接近，作为理想仿真用参数。

生成独立的 40 s 合成运动：ω(t) = 2 sin(2π·0.5t) + 0.8 sin(2π·1.7t)，角加速度使用解析导数；合成扭矩 τ_virtual = J_virtual·α + B_virtual·ω。

- 无噪声合成数据：R² = ${f(syntheticResult.noiseless.R2,8)}。
- 在扭矩上叠加标准差为无噪声扭矩标准差 1% 的高斯噪声（固定随机种子 20261015）：R² = ${f(syntheticResult.noisy.R2,8)}。
- 对含噪合成数据重新辨识：J = ${f(syntheticResult.refit.p[0],9)}，B = ${f(syntheticResult.refit.p[1],9)}。
- 同一虚拟参数用于未修改的急停原始数据：R² = ${f(virtualMeasured.stop.R2)}；用于急停 100 ms 平均数据：R² = ${f(virtualMeasured.stop100ms.R2)}；用于 7 文件的全部原始数据：R² = ${f(virtualMeasured.pooled.R2)}。

接近 1 的数值仅针对按该理想模型生成的合成数据，是模型自洽性示例，不是实测辨识精度。synthetic_virtual.csv 中的 torque_ideal_Nm、torque_noisy_Nm 均为生成值，不是原始测量。

## 后续需要确认的信息

优先确认真实采样周期、通道同步与 I0–I2 含义、扭矩的测量/指令属性及轴端换算。若仍存在工况依赖，可考虑库仑摩擦、速度相关摩擦、驱动器动态或通道延迟，并用独立数据检验。仅凭目前结果不能断言某一种因素就是误差原因。
`;
fs.writeFileSync(path.join(out,'辨识报告.md'),report);
// Replace the preliminary scratch report with the verified final report.
fs.writeFileSync('yaw_parameter_identification_report.md',report);
console.log(JSON.stringify({global100ms:gp,transfer,virtualMeasured,syntheticResult},null,2));
