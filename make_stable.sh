git tag -d stable
git push origin :stable
git commit --allow-empty -m "stable tag"
git tag stable
git push origin master --tags
