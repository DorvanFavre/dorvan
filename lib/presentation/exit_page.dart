import 'package:dorvan/domain/entity/exit_page.dart';
import 'package:dorvan/presentation/aware_text.dart';
import 'package:dorvan/presentation/rive_animation.dart';
import 'package:flutter/material.dart';

class ExitPageView extends StatelessWidget {
  final ExitPage? exitPage;

  ExitPageView({@required this.exitPage});

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        Positioned(
          left: -50,
          top: 0,
          child: Text(
            exitPage?.title ?? '-',
            maxLines: 1,
            softWrap: false,
            overflow: TextOverflow.visible,
            style: TextStyle(
                color: Colors.white, fontSize: 250, fontFamily: 'Futura'),
          ),
        ),
        Container(
          child: Center(
            child: Column(
              mainAxisSize: MainAxisSize.min,
              children: [
                GestureDetector(
                  onTap: () {
                    Navigator.of(context).pop();
                  },
                  child: Container(
                    height: 150,
                    width: 150,
                    child: RiveAnimation(
                      animation: '007_exit.riv',
                    ),
                  ),
                ),
                SizedBox(
                    //height: 100,
                    ),
                AwareText(
                  'article_exit',
                  builder: (text) => Text(
                    text,
                    style: TextStyle(fontSize: 16, color: Colors.black54),
                  ),
                )
              ],
            ),
          ),
        ),
      ],
    );
  }
}
