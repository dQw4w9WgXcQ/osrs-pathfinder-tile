FROM rust as build
WORKDIR /workdir
COPY . .
RUN cargo build --package service --release

FROM ubuntu
WORKDIR /workdir
COPY --from=build /workdir/target/release/service service
EXPOSE 8080
CMD ["./service"]