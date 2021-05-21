library('magick')
library(sqldf)
  require('magick')

##### change paths/labels/params here #####


paths = c(     'plastic_big_dvh',
		'plastic_big_dvh_c',
               'plastic_big_dvh_c_h_33',
                'plastic_big_dvh_c_h_7'
               )

environments = list(c( 'plane'),c( 'plane'),c( 'plane'),c( 'plane')
)

colors = list( c( '#B3FFBF','#FFDD99'),c( '#B3FFBF','#FFDD99'),c( '#B3FFBF','#FFDD99'),c( '#B3FFBF','#FFDD99'),c( '#B3FFBF','#FFDD99'),c( '#B3FFBF','#FFDD99'),c( '#B3FFBF','#FFDD99'),c( '#B3FFBF','#FFDD99'),c( '#B3FFBF','#FFDD99'),c( '#B3FFBF','#FFDD99'),c( '#B3FFBF','#FFDD99'),c( '#B3FFBF','#FFDD99'))

base_directory <- paste('data/', sep='')

runs = list( c(1:10),c(1:10),c(1:10),c(1:10))
gens = 99
pop = 100
num_top = 1

analysis = 'best'

##### change paths/labels/params here #####

output_directory = paste(base_directory,"../",analysis, sep='')


file <-file(paste(output_directory,'/best.txt',sep=''), open="w")

# for each method
for(m in 1:length(paths))
{
    path2 = paste("plots/path_best_",m,".jpeg",sep='')
    jpeg(file=path2)
    flag = 0
    # for each repetition
    for (exp in runs[[m]])
    	
      {
        input_paths  <-    paste(base_directory, paths[m],'_',exp, '/data_fullevolution/descriptors/',sep='')
        input_directory1  <-    paste(base_directory, paths[m],'_',exp, '/data_fullevolution/',environments[[m]][1],sep='')
        input_directory2  <-    paste(base_directory, paths[m],'_',exp, '/selectedpop_', sep='')

        ids_gens = data.frame()
        list = strsplit(list.files(paste(input_directory2, environments[[m]][1],'/selectedpop_',gens-1, sep='')), ' ')
        for(geno in 1:pop)
        {
          genome =  data.frame(cbind(c(gens-1), c(strsplit(strsplit(list [[geno]],'_')[[1]][3],'.png')[[1]][1] )))
          names(genome)<-c('generation','robot_id')
          ids_gens = rbind(ids_gens,genome)
        }
        measures   =  read.table(paste(input_directory1,"/all_measures.tsv", sep=''), header = TRUE, fill=TRUE)
        
        measures = sqldf(paste("select * from measures where displacement_velocity_hill <= 0.1"))
        
        bests =  sqldf(paste("select robot_id, fitness, displacement_velocity_hill from measures inner join ids_gens using (robot_id) order by fitness desc limit",num_top))
        
     
        # best fit in gen
      #  bests =  sqldf(paste("select robot_id, cons_fitness from measures inner join ids_gens using (robot_id, generation) order by cons_fitness desc limit",num_top))

        for(b in 1:nrow(bests))
        {
      
           writeLines( paste(paths[m],'exp',exp,bests[b,'robot_id'] ,bests[b,'fitness'] ), file )
          print( paste(paths[m],'exp',exp,bests[b,'robot_id'] ,bests[b,'fitness'],bests[b,'displacement_velocity_hill'] ))
   
           phenotype= bests[b,'robot_id'] 
       
           for (env in 1:length(environments[[m]]))
           {
               patha = paste(input_directory2, environments[[m]][env], "/selectedpop_",gens-1,sep="")
               
               body <- list.files(patha, paste("body_robot_",phenotype,".png$",sep=""), full.names = TRUE)
               body = image_read(body)
               body = image_scale(body, "400x400")
               body = image_border(image_background(body, colors[[m]][env]), "white", "5x5")
               
              if(b == 1 && env == 1)
              {
                bodies = body
              }else{
                bodies = c(bodies, body)
              }
              if(file.exists(paste(input_paths,'/positions_robot_',phenotype,'.txt',sep='')))
	      {
              	a=read.csv(paste(input_paths,'/positions_robot_',phenotype,'.txt',sep=''),sep=' ')
		x = unlist(a[6])
		y = unlist(a[7])
		

		if(flag == 0){
	    		plot(x,y,type='l',xlim=c(-1.75,1.75),ylim=c(-1,2.5),col=phenotype, cex = 1,panel.first = grid(10, lty = 1, lwd = 2))
	    		flag = 1
	  	}
	  	else{
	    		points(x,y,type='l',col=phenotype, cex = 1)
	    		line(x,y)
	  	}
		#dev.off()
		
		Dir <- paste("plots/",m,sep='')

		if (!file.exists(Dir)){
		    dir.create(file.path(Dir))
		}
		file.copy(paste(input_directory1,'/phenotypes/robot_',phenotype,'.yaml',sep=''), paste('plots/phenotypes/',m,'_',exp,'_','robot_',phenotype,'.yaml',sep=''))
		file.copy(paste(input_paths,'/positions_robot_',phenotype,'.txt',sep=''), Dir)
		path = paste("plots/",m,"/run",exp,"_path_robot_",phenotype,".jpeg",sep='')
		#jpeg(file=path)
		#print(path)
		#plot(x,y,type='b',xlim=c(-1.75,1.75),ylim=c(-1,2.5),col="red")


	      }
          }

        }
        
        if (length(environments[[m]])>1){
          env_file_name = 'seasons'
        }else{
          env_file_name = environments[[m]][env]
        }
          
        side_by_side = image_append(bodies, stack=F)
        image_write(side_by_side, path = paste(output_directory,"/",paths[m],'_', env_file_name, "_bodies_best_",exp,".png",sep=''), format = "png")
        
    }
            dev.off()
}


close(file)

