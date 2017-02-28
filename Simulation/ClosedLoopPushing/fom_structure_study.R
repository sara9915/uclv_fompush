setwd('~/MCubeLab/fompush/Simulation/ClosedLoopPushing')
data <- read.csv('SimulationResults/TestFOMStructure/cost_data_1000.csv', header = FALSE)
nbreaks <- 28
index <- seq(from = nbreaks, to = 0)
base <- 1.2
breakp <- c(0,exp(log(base) * (log(25)/log(base) - index)))
for(i in seq(from = 1, to = 100)){
  print(i)
  plot(hist(data[data$V2==i,]$V3))#, breaks=breakp))
  invisible(readline(prompt="Press [enter] to continue"))
}

normalize <- function()

for(i in seq(from = 1, to = 500)){
  print(i)
  plot(hist(data[data$V1==i,]$V3))
  invisible(readline(prompt="Press [enter] to continue"))
}

# Normalization
ndata <- data
for(i in seq(from = 1, to = 1000)){
  b <- data[data$V1==i,]
  m <- min(b$V3)
  M <- max(b$V3)
  b$V3 <- (b$V3 - m)/(M - m)
  ndata[ndata$V1==i,] <- b
}
pbreak = seq(from = 0, to = 1, by = 0.1)
for(i in seq(from = 1, to = 243)){
  print(i)
  plot(hist(ndata[ndata$V2==i,]$V3, breaks=pbreak))
  invisible(readline(prompt="Press [enter] to continue"))
}
# Standarization
sdata <- data
for(i in seq(from = 1, to = 1000)){
  b <- data[data$V1==i,]
  m <- mean(b$V3)
  d <- sd(b$V3)
  b$V3 <- (b$V3 - m)/v
  sdata[sdata$V1==i,] <- b
}
after_mean <- vector(mode = "numeric", length = 0L)
for(i in seq(from = 1, to = 243)){
  print(i)
  after_mean = c(after_mean, mean(sdata[sdata$V2==i,]$V3))
}
best_modes <- order(after_mean)
best_5 <- best_modes[1:5]
