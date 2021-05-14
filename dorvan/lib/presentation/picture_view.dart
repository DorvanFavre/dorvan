import 'package:dorvan/domain/entity/picture.dart';
import 'package:flutter/material.dart';

class PictureView extends StatelessWidget {
  final Picture? picture;

  PictureView({@required this.picture});

  @override
  Widget build(BuildContext context) {
    return Container(
      decoration: BoxDecoration(
          gradient: LinearGradient(
              colors: [
                Colors.black.withOpacity(0.8),
                Colors.black.withOpacity(0)
              ],
              begin: Alignment.topCenter,
              end: Alignment.bottomCenter,
              stops: [0, 1])),
      child: Image.asset(
        'assets/images/${picture!.imageName}',
        fit: BoxFit.cover,
      ),
    );
  }
}
