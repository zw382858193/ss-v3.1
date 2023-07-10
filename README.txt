git init
git add .
git remote add origin url(copy form GitHub Link)
git push -u origin master

切换分支
git checkout name

将缓存改为500M 解决error 10054
git config http.postBuffer 524288000

git config --global user.name "zhongw"
git config --global user.email "382858193@qq.com"
git clone url(copy form GitHub Link) 克隆
git init git 

提交分支
git checkout 分支名
git add .
git commit -m "xxx"

合并分支
git checkout 被合并分支(main)
git merge 合并分支(slave)

本地仓库推送到github
git add .
git commit -m "xxxx"
git push -u origin 分支名

远程仓库合并本地分支
git pull origin 远程分支名:本地分支名