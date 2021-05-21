for (robot in 1:10050)
{
	if(file.exists(paste('descriptors/positions_robot_',robot,'.txt',sep='')))
	{
		print(paste('descriptors/positions_robot_',robot,'.txt',sep=''))
		a=read.csv(paste('descriptors/positions_robot_',robot,'.txt',sep=''),sep=' ')
		x = unlist(a[6])
		y = unlist(a[7])
		path = paste("plots/path_robot_",robot,".jpeg",sep='')
		jpeg(file=path)
		print(path)
		plot(x,y,type='b')
		dev.off()
	}
	else{
		print("not found...")
	}
}	
