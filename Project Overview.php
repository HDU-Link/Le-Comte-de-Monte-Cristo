<?php
$title = "多智能体稳定控制、群集与绕环运动";
$files = [
    [
        "name" => "1 Jurdjevic-Quinn feedback control.py",
        "desc" => "实现反馈控制算法，抑制速度发散，保证多智能体系统稳定"
    ],
    [
        "name" => "2 Flocking transition.py",
        "desc" => "控制群集平滑转向，实现方向渐变过渡与速度一致性"
    ],
    [
        "name" => "3 Mill ring stability.py",
        "desc" => "验证绕环（Mill Ring）结构收敛性，分析半径与受力稳定性"
    ],
    [
        "name" => "4 Stabilization towards mill ring.py",
        "desc" => "通过最优控制输入，驱动智能体自动形成稳定环向结构"
    ],
    [
        "name" => "5 Mill to flock.py",
        "desc" => "实现绕环运动到群集运动的动态切换与控制"
    ]
];
$tech = "Python • NumPy • Matplotlib • SciPy";
?>

<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title><?= $title ?></title>
    <style>
        *{
            margin:0;
            padding:0;
            box-sizing:border-box;
            font-family: "Microsoft YaHei", "Segoe UI", sans-serif;
        }
        body{
            background-color: #f5f7fa;
            line-height: 1.6;
        }
        .container{
            max-width:720px;
            margin:40px auto;
            padding:0 20px;
        }
        .title{
            border-left:6px solid #2d8cf0;
            padding:12px 20px;
            color:#2a3b4c;
            font-size: 24px;
            font-weight: 600;
            margin-bottom: 20px;
            background: #fff;
            box-shadow: 0 2px 8px rgba(0,0,0,0.05);
        }
        .info{
            margin:20px 0;
        }
        .tag{
            background:#e8f3ff;
            color:#2d8cf0;
            padding:6px 12px;
            border-radius: 20px;
            font-size:14px;
            display: inline-block;
            font-weight: 500;
        }
        .item{
            background:#fff;
            padding:14px;
            margin:12px 0;
            border-radius:10px;
            box-shadow: 0 3px 10px rgba(0,0,0,0.04);
            transition: all 0.2s ease;
            border-left: 4px solid transparent;
        }
        .item:hover{
            transform: translateY(-2px);
            box-shadow: 0 5px 15px rgba(0,0,0,0.07);
            border-left-color: #2d8cf0;
        }
        .item h3{
            color:#2d8cf0;
            margin-bottom:8px;
            font-size: 16px;
            font-weight: 600;
        }
        .item p{
            color:#555;
            font-size: 14px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1 class="title"><?= $title ?></h1>
        <p class="info"><span class="tag"><?= $tech ?></span></p>
        
        <?php foreach($files as $f): ?>
        <div class="item">
            <h3><?= $f["name"] ?></h3>
            <p><?= $f["desc"] ?></p>
        </div>
        <?php endforeach; ?>
    </div>
</body>
</html>